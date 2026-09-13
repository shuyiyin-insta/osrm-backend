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

#include <cstdint>
#include <utility>
#include <vector>

namespace osrm::engine::routing_algorithms
{

enum class ReachabilitySearchStatus : std::uint8_t
{
    Complete,
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
    ReachabilitySearchStatus status = ReachabilitySearchStatus::Complete;

    bool isComplete() const { return status == ReachabilitySearchStatus::Complete; }
};

namespace mld
{

template <typename FacadeT>
ReachabilitySearchResult reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
                                            const FacadeT &facade,
                                            const PhantomNodeCandidates &source_candidates,
                                            const EdgeDuration max_duration)
{
    BOOST_ASSERT(max_duration >= EdgeDuration{0});

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
            relaxOutgoingEdges<FORWARD_DIRECTION>(
                facade, query_heap, source_node, ReachabilitySearch{});
        }
    };

    for (const auto &source : source_candidates)
    {
        if (source.IsValidForwardSource())
        {
            insert_source(source.forward_segment_id.id,
                          source.GetForwardWeightAsSource(),
                          source.GetForwardDurationAsSource());
        }
        if (source.IsValidReverseSource())
        {
            insert_source(source.reverse_segment_id.id,
                          source.GetReverseWeightAsSource(),
                          source.GetReverseDurationAsSource());
        }
    }

    ReachabilitySearchResult result;
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

        relaxOutgoingEdges<FORWARD_DIRECTION>(facade, query_heap, heap_node, ReachabilitySearch{});
    }

    return result;
}

} // namespace mld

template <typename Algorithm>
ReachabilitySearchResult reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
                                            const DataFacade<Algorithm> &facade,
                                            const PhantomNodeCandidates &source_candidates,
                                            EdgeDuration max_duration);

template <>
inline ReachabilitySearchResult
reachabilitySearch<mld::Algorithm>(SearchEngineData<mld::Algorithm> &engine_working_data,
                                   const DataFacade<mld::Algorithm> &facade,
                                   const PhantomNodeCandidates &source_candidates,
                                   const EdgeDuration max_duration)
{ return mld::reachabilitySearch(engine_working_data, facade, source_candidates, max_duration); }

template <>
inline ReachabilitySearchResult
reachabilitySearch<ch::Algorithm>(SearchEngineData<ch::Algorithm> &engine_working_data,
                                  const DataFacade<ch::Algorithm> &facade,
                                  const PhantomNodeCandidates &source_candidates,
                                  const EdgeDuration max_duration)
{
    static_cast<void>(engine_working_data);
    auto ch_result = ch::phastOneToAllSearch(facade, source_candidates, max_duration);

    ReachabilitySearchResult result;
    result.nodes.reserve(ch_result.nodes.size());
    for (const auto &node : ch_result.nodes)
        result.nodes.push_back({node.node, node.weight, node.duration});

    switch (ch_result.status)
    {
    case ch::IsochroneSearchStatus::Complete:
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
