#ifndef OSRM_ENGINE_REACHABILITY_HPP
#define OSRM_ENGINE_REACHABILITY_HPP

#include "engine/algorithm.hpp"
#include "engine/datafacade.hpp"
#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/routing_base_mld.hpp"
#include "engine/search_engine_data.hpp"

#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <vector>

namespace osrm::engine::routing_algorithms
{

struct ReachabilityResult
{
    NodeID node;
    EdgeWeight weight;
    EdgeDuration duration;
};

namespace mld
{

template <typename FacadeT>
std::vector<ReachabilityResult> reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
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

    std::vector<ReachabilityResult> reachable_nodes;
    while (!query_heap.Empty())
    {
        const auto heap_node = query_heap.DeleteMinGetHeapNode();

        // Weight is the optimization metric.  An over-duration, lower-weight label still needs
        // to propagate: it can suppress an in-cutoff but higher-weight competing path.  The
        // duration cutoff is therefore applied only to returned labels.
        if (heap_node.data.duration <= max_duration)
        {
            reachable_nodes.push_back({heap_node.node, heap_node.weight, heap_node.data.duration});
        }

        relaxOutgoingEdges<FORWARD_DIRECTION>(facade, query_heap, heap_node, ReachabilitySearch{});
    }

    return reachable_nodes;
}

} // namespace mld

template <typename Algorithm>
std::vector<ReachabilityResult> reachabilitySearch(SearchEngineData<Algorithm> &engine_working_data,
                                                   const DataFacade<Algorithm> &facade,
                                                   const PhantomNodeCandidates &source_candidates,
                                                   EdgeDuration max_duration);

template <>
inline std::vector<ReachabilityResult>
reachabilitySearch<mld::Algorithm>(SearchEngineData<mld::Algorithm> &engine_working_data,
                                   const DataFacade<mld::Algorithm> &facade,
                                   const PhantomNodeCandidates &source_candidates,
                                   const EdgeDuration max_duration)
{ return mld::reachabilitySearch(engine_working_data, facade, source_candidates, max_duration); }

} // namespace osrm::engine::routing_algorithms

#endif // OSRM_ENGINE_REACHABILITY_HPP
