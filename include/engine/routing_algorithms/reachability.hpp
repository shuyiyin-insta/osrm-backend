#ifndef OSRM_ENGINE_REACHABILITY_HPP
#define OSRM_ENGINE_REACHABILITY_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/isochrone_ch.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace osrm::engine::routing_algorithms
{

enum class ReachabilitySearchStatus : std::uint8_t
{
    Complete,
    SearchNodeLimitReached,
    UnsupportedGraph,
    ArithmeticOverflow
};

struct ReachabilityNode
{
    NodeID node;
    EdgeWeight weight;
    EdgeDuration duration;
};

struct ReachabilitySearchResult
{
    std::vector<ReachabilityNode> nodes;
    std::vector<ReachabilityNode> competitors;
    ReachabilitySearchStatus status = ReachabilitySearchStatus::Complete;

    bool isComplete() const { return status == ReachabilitySearchStatus::Complete; }
};

namespace mld
{

template <typename Metric> bool checkedAdd(const Metric lhs, const Metric rhs, Metric &result)
{
    using Value = typename Metric::value_type;
    const auto sum = static_cast<std::int64_t>(from_alias<Value>(lhs)) +
                     static_cast<std::int64_t>(from_alias<Value>(rhs));
    if (sum < std::numeric_limits<Value>::min() || sum >= std::numeric_limits<Value>::max())
        return false;
    result = Metric{static_cast<Value>(sum)};
    return true;
}

template <bool DIRECTION, typename FacadeT, typename Heap, typename HeapNodeT>
bool relaxReachabilityEdges(const FacadeT &facade, Heap &heap, const HeapNodeT &heap_node)
{
    for (const auto edge : facade.GetBorderEdgeRange(0, heap_node.node))
    {
        const auto traversable = DIRECTION == FORWARD_DIRECTION ? facade.IsForwardEdge(edge)
                                                                : facade.IsBackwardEdge(edge);
        if (!traversable)
            continue;

        const auto target = facade.GetTarget(edge);
        if (facade.ExcludeNode(target))
            continue;

        const auto &edge_data = facade.GetEdgeData(edge);
        const auto metric_node = DIRECTION == FORWARD_DIRECTION ? heap_node.node : target;
        EdgeWeight weight;
        if (!checkedAdd(heap_node.weight, facade.GetNodeWeight(metric_node), weight) ||
            !checkedAdd(weight,
                        alias_cast<EdgeWeight>(facade.GetWeightPenaltyForEdgeID(edge_data.turn_id)),
                        weight))
            return false;

        EdgeDuration duration;
        if (!checkedAdd(heap_node.data.duration, facade.GetNodeDuration(metric_node), duration) ||
            !checkedAdd(
                duration,
                alias_cast<EdgeDuration>(facade.GetDurationPenaltyForEdgeID(edge_data.turn_id)),
                duration))
            return false;

        insertOrUpdate(heap, target, weight, {heap_node.node, false, duration});
    }
    return true;
}

template <bool DIRECTION, typename FacadeT>
ReachabilitySearchResult
reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
                   const FacadeT &facade,
                   const PhantomNodeCandidates &endpoint_candidates,
                   const EdgeDuration max_duration,
                   const std::size_t maximum_search_nodes = std::numeric_limits<std::size_t>::max())
{
    BOOST_ASSERT(max_duration >= EdgeDuration{0});

    ReachabilitySearchResult result;
    if (facade.GetNumberOfNodes() > maximum_search_nodes)
    {
        result.status = ReachabilitySearchStatus::SearchNodeLimitReached;
        return result;
    }

    engine_working_data.InitializeOrClearReachabilityThreadLocalStorage(
        facade.GetNumberOfNodes(), facade.GetMaxBorderNodeID() + 1);
    auto &query_heap = *engine_working_data.reachability_heap;

    struct SourceNode
    {
        NodeID node;
        EdgeWeight weight;
        ReachabilityMultiLayerDijkstraHeapData data;
    };

    const auto insert_source =
        [&](const NodeID node, const EdgeWeight weight, const EdgeDuration duration)
    {
        if (!facade.ExcludeNode(node))
        {
            // A source label is virtual: it starts at a partial segment and is therefore not a
            // label for the edge-based node itself.  Relax it directly into the query heap so a
            // real path that later returns to this node is not discarded as a worse duplicate.
            const SourceNode source_node{node, weight, {node, false, duration}};
            if (!relaxReachabilityEdges<DIRECTION>(facade, query_heap, source_node))
                result.status = ReachabilitySearchStatus::ArithmeticOverflow;
        }
    };

    for (const auto &endpoint : endpoint_candidates)
    {
        const auto forward_is_valid = DIRECTION == FORWARD_DIRECTION
                                          ? endpoint.IsValidForwardSource()
                                          : endpoint.IsValidForwardTarget();
        if (forward_is_valid)
        {
            insert_source(endpoint.forward_segment_id.id,
                          DIRECTION == FORWARD_DIRECTION ? endpoint.GetForwardWeightAsSource()
                                                         : endpoint.GetForwardWeightAsTarget(),
                          DIRECTION == FORWARD_DIRECTION ? endpoint.GetForwardDurationAsSource()
                                                         : endpoint.GetForwardDurationAsTarget());
        }
        const auto reverse_is_valid = DIRECTION == FORWARD_DIRECTION
                                          ? endpoint.IsValidReverseSource()
                                          : endpoint.IsValidReverseTarget();
        if (reverse_is_valid)
        {
            insert_source(endpoint.reverse_segment_id.id,
                          DIRECTION == FORWARD_DIRECTION ? endpoint.GetReverseWeightAsSource()
                                                         : endpoint.GetReverseWeightAsTarget(),
                          DIRECTION == FORWARD_DIRECTION ? endpoint.GetReverseDurationAsSource()
                                                         : endpoint.GetReverseDurationAsTarget());
        }
    }
    if (!result.isComplete())
        return result;

    while (!query_heap.Empty())
    {
        const auto heap_node = query_heap.DeleteMinGetHeapNode();

        // Weight is the optimization metric.  An over-duration, lower-weight label still needs
        // to propagate: it can suppress an in-cutoff but higher-weight competing path.  The
        // duration cutoff is therefore applied only to returned labels.
        if (heap_node.data.duration <= max_duration)
        {
            result.nodes.push_back({heap_node.node, heap_node.weight, heap_node.data.duration});
        }
        else
        {
            result.competitors.push_back(
                {heap_node.node, heap_node.weight, heap_node.data.duration});
        }

        if (!relaxReachabilityEdges<DIRECTION>(facade, query_heap, heap_node))
        {
            result.status = ReachabilitySearchStatus::ArithmeticOverflow;
            return result;
        }
    }

    return result;
}

template <typename FacadeT>
ReachabilitySearchResult reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
                                            const FacadeT &facade,
                                            const PhantomNodeCandidates &source_candidates,
                                            const EdgeDuration max_duration)
{
    return reachabilitySearch<FORWARD_DIRECTION>(
        engine_working_data, facade, source_candidates, max_duration);
}

} // namespace mld

template <typename Algorithm>
ReachabilitySearchResult
reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
                   const DataFacade<Algorithm> &facade,
                   const PhantomNodeCandidates &source_candidates,
                   EdgeDuration max_duration,
                   bool inbound = false,
                   std::size_t maximum_search_nodes = std::numeric_limits<std::size_t>::max());

template <>
inline ReachabilitySearchResult
reachabilitySearch<mld::Algorithm>(SearchEngineData<mld::Algorithm> &engine_working_data,
                                   const DataFacade<mld::Algorithm> &facade,
                                   const PhantomNodeCandidates &source_candidates,
                                   const EdgeDuration max_duration,
                                   const bool inbound,
                                   const std::size_t maximum_search_nodes)
{
    if (inbound)
        return mld::reachabilitySearch<REVERSE_DIRECTION>(
            engine_working_data, facade, source_candidates, max_duration, maximum_search_nodes);
    return mld::reachabilitySearch<FORWARD_DIRECTION>(
        engine_working_data, facade, source_candidates, max_duration, maximum_search_nodes);
}

template <>
inline ReachabilitySearchResult
reachabilitySearch<ch::Algorithm>(SearchEngineData<ch::Algorithm> &engine_working_data,
                                  const DataFacade<ch::Algorithm> &facade,
                                  const PhantomNodeCandidates &source_candidates,
                                  const EdgeDuration max_duration,
                                  const bool inbound,
                                  const std::size_t maximum_search_nodes)
{
    static_cast<void>(engine_working_data);
    auto ch_result = inbound ? ch::phastOneToAllSearch<false>(
                                   facade, source_candidates, max_duration, maximum_search_nodes)
                             : ch::phastOneToAllSearch<true>(
                                   facade, source_candidates, max_duration, maximum_search_nodes);

    ReachabilitySearchResult result;
    result.nodes.reserve(ch_result.nodes.size());
    for (const auto &node : ch_result.nodes)
        result.nodes.push_back({node.node, node.weight, node.duration});
    result.competitors.reserve(ch_result.competitors.size());
    for (const auto &node : ch_result.competitors)
        result.competitors.push_back({node.node, node.weight, node.duration});

    switch (ch_result.status)
    {
    case ch::IsochroneSearchStatus::Complete:
        break;
    case ch::IsochroneSearchStatus::SearchNodeLimitReached:
        result.status = ReachabilitySearchStatus::SearchNodeLimitReached;
        break;
    case ch::IsochroneSearchStatus::UnsupportedCHGraph:
        result.status = ReachabilitySearchStatus::UnsupportedGraph;
        break;
    case ch::IsochroneSearchStatus::ArithmeticOverflow:
        result.status = ReachabilitySearchStatus::ArithmeticOverflow;
        break;
    }
    return result;
}

} // namespace osrm::engine::routing_algorithms

#endif // OSRM_ENGINE_REACHABILITY_HPP
