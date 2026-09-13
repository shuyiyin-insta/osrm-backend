#include "engine/plugins/isochrone.hpp"

#include "engine/plugins/plugin_base.hpp"
#include "engine/routing_algorithms.hpp"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

namespace osrm
{
namespace engine
{
namespace plugins
{
namespace
{
thread_local std::unordered_map<std::uint64_t, std::vector<NodeID>> unpacked_path_cache;
thread_local std::uint32_t unpacked_path_cache_checksum = 0;
constexpr std::size_t MAX_UNPACKED_PATH_CACHE_ENTRIES = 50000;

struct GeometryLabel
{
    EdgeWeight weight;
    EdgeDuration duration;
};
} // namespace

Status IsochronePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                      const api::IsochroneParameters &parameters,
                                      osrm::engine::api::ResultT &result) const
{
    if (!parameters.IsValid())
        return Error("InvalidOptions", "Invalid isochrone parameters.", result);

    if (!CheckAllCoordinates(parameters.coordinates))
        return Error("InvalidOptions", "Coordinates are invalid", result);

    if (!CheckAlgorithms(parameters, algorithms, result))
        return Status::Error;

    BOOST_ASSERT(parameters.IsValid());

    const unsigned configured_max_range =
        max_range_seconds > 0 ? static_cast<unsigned>(max_range_seconds) : 1;
    const unsigned effective_range = std::min(parameters.range, configured_max_range);
    const EdgeDuration duration_threshold =
        to_alias<EdgeDuration>(static_cast<int>(effective_range * 10));

    const auto &facade = algorithms.GetFacade();
    const auto phantom_nodes = GetPhantomNodes(facade, parameters, 1);
    if (phantom_nodes.empty() || phantom_nodes.front().empty())
        return Error("NoSegment", "Could not find a matching segment for the coordinate.", result);

    const PhantomNodeCandidates source_candidates{phantom_nodes.front().front().phantom_node};
    const auto search_result =
        algorithms.IsochroneSearch(source_candidates, duration_threshold, false);
    if (search_result.status == routing_algorithms::ReachabilitySearchStatus::UnsupportedGraph)
        return Error(
            "NotImplemented", "The loaded graph does not support isochrone search.", result);
    if (search_result.status == routing_algorithms::ReachabilitySearchStatus::ArithmeticOverflow)
        return Error("InvalidValue", "Isochrone metric arithmetic overflowed.", result);

    BOOST_ASSERT(search_result.isComplete());

    std::stringstream buf;
    buf << std::setprecision(12);
    buf << "{\"type\":\"FeatureCollection\",\"max_range\":" << effective_range << ",\"features\":[";
    bool first_feature = true;
    std::unordered_map<std::uint64_t, GeometryLabel> geometry_labels;

    for (const auto &search_node : search_result.nodes)
    {
        const NodeID node = search_node.node;
        const auto duration = search_node.duration;

        // Defensive filter: only include nodes within threshold
        if (duration > duration_threshold)
            continue;

        // Map edge-based node -> full geometry (forward or reverse)
        const auto geom = facade.GetGeometryIndex(node);
        if (geom.id == SPECIAL_GEOMETRYID)
        {
            continue;
        }

        const std::uint64_t key =
            (static_cast<std::uint64_t>(geom.id) << 1u) | (geom.forward ? 1u : 0u);
        const auto found = geometry_labels.find(key);
        if (found == geometry_labels.end() || search_node.weight < found->second.weight)
        {
            geometry_labels[key] = {search_node.weight, duration};
        }
    }

    std::vector<std::pair<std::uint64_t, GeometryLabel>> sorted_edges(geometry_labels.begin(),
                                                                      geometry_labels.end());
    std::sort(sorted_edges.begin(),
              sorted_edges.end(),
              [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });

    const auto current_checksum = facade.GetCheckSum();
    if (unpacked_path_cache_checksum != current_checksum)
    {
        unpacked_path_cache.clear();
        unpacked_path_cache_checksum = current_checksum;
    }

    const auto get_unpacked_geometry_nodes = [&](const PackedGeometryID geometry_id,
                                                 const bool forward) -> const std::vector<NodeID> &
    {
        const std::uint64_t cache_key = (static_cast<std::uint64_t>(geometry_id) << 1u) |
                                        static_cast<std::uint64_t>(forward ? 1u : 0u);
        const auto found = unpacked_path_cache.find(cache_key);
        if (found != unpacked_path_cache.end())
            return found->second;

        if (unpacked_path_cache.size() >= MAX_UNPACKED_PATH_CACHE_ENTRIES)
        {
            unpacked_path_cache.clear();
        }

        auto [it, _inserted] = unpacked_path_cache.emplace(cache_key, std::vector<NodeID>{});
        auto &nodes = it->second;
        if (forward)
        {
            for (const auto node_id : facade.GetUncompressedForwardGeometry(geometry_id))
                nodes.push_back(node_id);
        }
        else
        {
            for (const auto node_id : facade.GetUncompressedReverseGeometry(geometry_id))
                nodes.push_back(node_id);
        }
        return nodes;
    };

    for (const auto &entry : sorted_edges)
    {
        const auto key = entry.first;
        const auto &label = entry.second;
        const auto geometry_id = static_cast<PackedGeometryID>(key >> 1u);
        const bool forward = (key & 1u) == 1u;

        std::stringstream geometry_buf;
        geometry_buf << std::setprecision(12);
        geometry_buf << "{\"type\":\"Feature\",\"properties\":{\"weight\":"
                     << from_alias<int>(label.weight)
                     << ",\"duration\":" << from_alias<int>(label.duration)
                     << "},\"geometry\":{\"type\":\"LineString\",\"coordinates\":[";

        const auto &geometry_nodes = get_unpacked_geometry_nodes(geometry_id, forward);
        bool first_coordinate = true;
        std::size_t coordinate_count = 0;
        const auto append_coordinate = [&](const NodeID node_id)
        {
            const auto coord = facade.GetCoordinateOfNode(node_id);
            if (!first_coordinate)
                geometry_buf << ',';
            first_coordinate = false;
            geometry_buf << '[' << toFloating(coord.lon) << ',' << toFloating(coord.lat) << ']';
            ++coordinate_count;
        };

        for (const auto node_id : geometry_nodes)
            append_coordinate(node_id);

        if (coordinate_count < 2)
            continue;

        geometry_buf << "]}}";
        if (!first_feature)
            buf << ',';
        first_feature = false;
        buf << geometry_buf.str();
    }

    buf << "]}";

    result = std::string(buf.str());
    return Status::Ok;
}

} // namespace plugins
} // namespace engine
} // namespace osrm
