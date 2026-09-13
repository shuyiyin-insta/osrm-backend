#include "engine/isochrone/native_search_result.hpp"

#include "util/typedefs.hpp"

#include <algorithm>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_set>

namespace osrm::engine::isochrone
{
namespace
{

std::optional<std::int64_t> geometryDuration(const datafacade::BaseDataFacade &facade,
                                             const NodeID node)
{
    const auto geometry = facade.GetGeometryIndex(node);
    if (geometry.id == SPECIAL_GEOMETRYID)
        return std::nullopt;

    const auto sum_durations = [](const auto &durations) -> std::optional<std::int64_t>
    {
        std::int64_t total = 0;
        for (const auto duration : durations)
        {
            if (duration == INVALID_SEGMENT_DURATION)
                return std::nullopt;
            total += from_alias<std::uint32_t>(duration);
            if (total >= std::numeric_limits<EdgeDuration::value_type>::max())
                return std::nullopt;
        }
        return total;
    };

    if (geometry.forward)
        return sum_durations(facade.GetUncompressedForwardDurations(geometry.id));
    return sum_durations(facade.GetUncompressedReverseDurations(geometry.id));
}

std::optional<bool> intersectsCutoff(const datafacade::BaseDataFacade &facade,
                                     const routing_algorithms::ReachabilityNode &label,
                                     const EdgeDuration duration_cutoff,
                                     const bool inbound)
{
    if (!inbound)
        return label.duration <= duration_cutoff;

    const auto geometry_duration = geometryDuration(facade, label.node);
    if (!geometry_duration)
        return std::nullopt;
    const auto last_duration = from_alias<std::int64_t>(label.duration) - *geometry_duration;
    return last_duration <= from_alias<std::int64_t>(duration_cutoff);
}

bool isValidMetric(const EdgeWeight metric) { return metric != INVALID_EDGE_WEIGHT; }

bool isValidMetric(const EdgeDuration metric)
{ return metric != INVALID_EDGE_DURATION && metric != MAXIMAL_EDGE_DURATION; }

} // namespace

SearchResult
makeNativeSearchResult(const datafacade::BaseDataFacade &facade,
                       const routing_algorithms::ReachabilitySearchResult &native_result,
                       const PhantomNodeCandidates &endpoint_candidates,
                       const EdgeDuration duration_cutoff,
                       const std::size_t maximum_records,
                       const bool inbound)
{
    SearchResult result;
    if (!native_result.isComplete())
    {
        result.status = SearchStatus::ArithmeticOverflow;
        return result;
    }

    const auto reserve_record = [&]()
    {
        const auto used =
            result.nodes.size() + result.competitors.size() + result.phantom_partials.size();
        if (used >= maximum_records)
        {
            result.status = SearchStatus::SearchRecordLimitReached;
            return false;
        }
        return true;
    };

    std::unordered_set<PackedGeometryID> candidate_geometries;
    const auto append_partial =
        [&](const PhantomNode &phantom, const NodeID node, const bool forward)
    {
        if (!result.isComplete() || facade.ExcludeNode(node))
            return;
        if (!reserve_record())
            return;

        const auto seed_weight = inbound ? (forward ? phantom.GetForwardWeightAsTarget()
                                                    : phantom.GetReverseWeightAsTarget())
                                         : (forward ? phantom.GetForwardWeightAsSource()
                                                    : phantom.GetReverseWeightAsSource());
        const auto seed_duration = inbound ? (forward ? phantom.GetForwardDurationAsTarget()
                                                      : phantom.GetReverseDurationAsTarget())
                                           : (forward ? phantom.GetForwardDurationAsSource()
                                                      : phantom.GetReverseDurationAsSource());
        if (!isValidMetric(seed_weight) || !isValidMetric(seed_duration) ||
            !isValidMetric(phantom.approach_weight) || !isValidMetric(phantom.approach_duration))
        {
            result.status = SearchStatus::ArithmeticOverflow;
            return;
        }

        const auto reaches_network = phantom.approach_duration <= duration_cutoff;
        result.phantom_partials.push_back(
            {node,
             phantom,
             forward ? PhantomTraversalDirection::Forward : PhantomTraversalDirection::Reverse,
             inbound ? PhantomTraversalKind::InboundTerminal
                     : PhantomTraversalKind::OutboundInitial,
             seed_duration,
             reaches_network,
             seed_weight});

        if (reaches_network)
        {
            const auto geometry = facade.GetGeometryIndex(node);
            if (geometry.id == SPECIAL_GEOMETRYID)
                result.status = SearchStatus::ArithmeticOverflow;
            else
                candidate_geometries.insert(geometry.id);
        }
    };

    for (const auto &endpoint : endpoint_candidates)
    {
        if ((!inbound && endpoint.IsValidForwardSource()) ||
            (inbound && endpoint.IsValidForwardTarget()))
            append_partial(endpoint, endpoint.forward_segment_id.id, true);
        if ((!inbound && endpoint.IsValidReverseSource()) ||
            (inbound && endpoint.IsValidReverseTarget()))
            append_partial(endpoint, endpoint.reverse_segment_id.id, false);
    }
    if (!result.isComplete())
        return result;

    const auto visit_native_labels = [&](const auto &visitor)
    {
        for (const auto &label : native_result.nodes)
            visitor(label);
        for (const auto &label : native_result.competitors)
            visitor(label);
    };

    visit_native_labels(
        [&](const routing_algorithms::ReachabilityNode &label)
        {
            if (!result.isComplete())
                return;
            const auto geometry = facade.GetGeometryIndex(label.node);
            if (geometry.id == SPECIAL_GEOMETRYID)
                return;
            const auto intersects = intersectsCutoff(facade, label, duration_cutoff, inbound);
            if (!intersects)
            {
                result.status = SearchStatus::ArithmeticOverflow;
                return;
            }
            if (*intersects)
                candidate_geometries.insert(geometry.id);
        });
    if (!result.isComplete())
        return result;

    visit_native_labels(
        [&](const routing_algorithms::ReachabilityNode &label)
        {
            if (!result.isComplete())
                return;
            const auto geometry = facade.GetGeometryIndex(label.node);
            if (geometry.id == SPECIAL_GEOMETRYID || !candidate_geometries.contains(geometry.id))
                return;

            const auto intersects = intersectsCutoff(facade, label, duration_cutoff, inbound);
            if (!intersects)
            {
                result.status = SearchStatus::ArithmeticOverflow;
                return;
            }
            if (!reserve_record())
                return;

            const SearchResult::Node node{
                label.node, label.duration, NodeProvenance::Network, label.weight};
            if (*intersects)
                result.nodes.push_back(node);
            else
                result.competitors.push_back(node);
        });

    return result;
}

} // namespace osrm::engine::isochrone
