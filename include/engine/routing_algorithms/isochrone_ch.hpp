#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP

#include "engine/phantom_node.hpp"

#include "util/integer_range.hpp"
#include "util/query_heap.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

namespace osrm::engine::routing_algorithms::ch
{

enum class IsochroneSearchStatus : std::uint8_t
{
    Complete,
    // The query graph references a node outside its declared node range.
    UnsupportedCHGraph,
    ArithmeticOverflow
};

struct IsochroneSearchNode
{
    NodeID node;
    EdgeWeight weight;
    EdgeDuration duration;
};

struct IsochroneSearchResult
{
    std::vector<IsochroneSearchNode> nodes;
    IsochroneSearchStatus status = IsochroneSearchStatus::Complete;

    bool isComplete() const { return status == IsochroneSearchStatus::Complete; }
};

namespace detail
{

struct IsochroneCHHeapData
{
    EdgeDuration duration;
};

using IsochroneCHQueryHeap = util::QueryHeap<NodeID,
                                             NodeID,
                                             EdgeWeight,
                                             IsochroneCHHeapData,
                                             util::UnorderedMapStorage<NodeID, int>>;

struct IsochroneCHNodeLabel
{
    EdgeWeight weight = INVALID_EDGE_WEIGHT;
    EdgeDuration duration = MAXIMAL_EDGE_DURATION;
    bool reachable = false;
};

// A backward CH arc is stored at its lower endpoint, but represents travel from its target to
// that endpoint.  A retained core has no PHAST rank, so its backward arcs need an incoming index
// for its local Dijkstra search.
struct CoreBackwardEdge
{
    NodeID target;
    EdgeWeight weight;
    EdgeDuration duration;
};

using CoreBackwardEdges = std::vector<std::vector<CoreBackwardEdge>>;

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

template <typename Metric> bool isValidMetric(const Metric metric)
{
    using Value = typename Metric::value_type;
    return from_alias<Value>(metric) < std::numeric_limits<Value>::max();
}

inline bool isBetterLabel(const EdgeWeight candidate_weight,
                          const EdgeDuration candidate_duration,
                          const IsochroneCHNodeLabel &current)
{
    // CH preprocessing chooses paths by profile weight.  Its shortcuts can have discarded a
    // different equal-weight path with a shorter duration, so duration is carried metadata, not
    // a secondary shortest-path criterion.
    static_cast<void>(candidate_duration);
    return !current.reachable || candidate_weight < current.weight;
}

inline bool isBetterLabel(const EdgeWeight candidate_weight,
                          const EdgeDuration candidate_duration,
                          const IsochroneCHQueryHeap::HeapNode &current)
{
    static_cast<void>(candidate_duration);
    return candidate_weight < current.weight;
}

template <typename CHFacade>
bool buildDownwardOrder(const CHFacade &facade,
                        std::vector<NodeID> &order,
                        CoreBackwardEdges &core_backward_edges)
{
    const auto number_of_nodes = facade.GetNumberOfNodes();
    std::vector<unsigned> indegree(number_of_nodes, 0);

    // In a fully contracted CH every non-self stored arc points from a lower-ranked node to a
    // higher-ranked node.  Backward-only arcs matter too: the PHAST downward sweep reads them
    // from the lower-ranked endpoint.
    for (const auto node : util::irange<NodeID>(0, number_of_nodes))
    {
        for (const auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            const auto target = facade.GetTarget(edge);
            if (target >= number_of_nodes)
                return false;
            if (target != node && (data.forward || data.backward))
                ++indegree[target];
        }
    }

    std::queue<NodeID> ready;
    for (const auto node : util::irange<NodeID>(0, number_of_nodes))
    {
        if (indegree[node] == 0)
            ready.push(node);
    }

    order.clear();
    order.reserve(number_of_nodes);
    while (!ready.empty())
    {
        const auto node = ready.front();
        ready.pop();
        order.push_back(node);

        for (const auto edge : facade.GetAdjacentEdgeRange(node))
        {
            const auto &data = facade.GetEdgeData(edge);
            const auto target = facade.GetTarget(edge);
            if (target == node || !(data.forward || data.backward))
                continue;

            BOOST_ASSERT(indegree[target] > 0);
            --indegree[target];
            if (indegree[target] == 0)
                ready.push(target);
        }
    }

    // Excludable CH graphs retain a core.  It has no usable PHAST rank, so search its logical
    // backward arcs with Dijkstra.  The ordered portion remains a normal downward PHAST sweep.
    core_backward_edges.clear();
    core_backward_edges.resize(number_of_nodes);
    for (const auto lower : util::irange<NodeID>(0, number_of_nodes))
    {
        if (indegree[lower] == 0)
            continue;

        for (const auto edge : facade.GetAdjacentEdgeRange(lower))
        {
            const auto &data = facade.GetEdgeData(edge);
            const auto higher = facade.GetTarget(edge);
            if (data.backward && indegree[higher] != 0)
            {
                core_backward_edges[higher].push_back(
                    {lower, data.weight, to_alias<EdgeDuration>(data.duration)});
            }
        }
    }

    return true;
}

inline bool relaxLabel(const IsochroneCHNodeLabel &from,
                       const EdgeWeight edge_weight,
                       const EdgeDuration edge_duration,
                       IsochroneCHNodeLabel &to)
{
    BOOST_ASSERT(from.reachable);
    BOOST_ASSERT(edge_weight > EdgeWeight{0});
    BOOST_ASSERT(edge_duration >= EdgeDuration{0});

    EdgeWeight weight;
    EdgeDuration duration;
    if (!checkedAdd(from.weight, edge_weight, weight) ||
        !checkedAdd(from.duration, edge_duration, duration))
        return false;

    if (isBetterLabel(weight, duration, to))
        to = {weight, duration, true};
    return true;
}

inline void insertOrUpdate(IsochroneCHQueryHeap &heap,
                           const NodeID node,
                           const EdgeWeight weight,
                           const EdgeDuration duration)
{
    const auto current = heap.GetHeapNodeIfWasInserted(node);
    if (current == nullptr)
    {
        heap.Insert(node, weight, {duration});
        return;
    }

    if (current->WasRemoved() || !isBetterLabel(weight, duration, *current))
        return;

    current->data.duration = duration;
    if (weight < current->weight)
    {
        current->weight = weight;
        heap.DecreaseKey(*current);
    }
}

inline bool relaxCoreBackwardEdges(IsochroneCHQueryHeap &heap,
                                   const IsochroneCHNodeLabel &from,
                                   const std::vector<CoreBackwardEdge> &edges)
{
    for (const auto &edge : edges)
    {
        BOOST_ASSERT(from.reachable);
        BOOST_ASSERT(edge.weight > EdgeWeight{0});
        BOOST_ASSERT(edge.duration >= EdgeDuration{0});

        EdgeWeight weight;
        EdgeDuration duration;
        if (!checkedAdd(from.weight, edge.weight, weight) ||
            !checkedAdd(from.duration, edge.duration, duration))
            return false;

        insertOrUpdate(heap, edge.target, weight, duration);
    }
    return true;
}

inline bool insertSource(std::vector<IsochroneCHNodeLabel> &source_labels,
                         const NodeID node,
                         const EdgeWeight weight,
                         const EdgeDuration duration)
{
    if (!isValidMetric(weight) || !isValidMetric(duration))
        return false;

    auto &current = source_labels[node];
    if (isBetterLabel(weight, duration, current))
        current = {weight, duration, true};
    return true;
}

template <typename CHFacade>
bool initializeSources(IsochroneCHQueryHeap &heap,
                       std::vector<IsochroneCHNodeLabel> &source_labels,
                       const PhantomNodeCandidates &source_candidates,
                       const CHFacade &facade,
                       const CoreBackwardEdges &core_backward_edges)
{
    const auto number_of_nodes = facade.GetNumberOfNodes();
    for (const auto &source : source_candidates)
    {
        if (source.IsValidForwardSource())
        {
            if (source.forward_segment_id.id >= number_of_nodes ||
                !insertSource(source_labels,
                              source.forward_segment_id.id,
                              source.GetForwardWeightAsSource(),
                              source.GetForwardDurationAsSource()))
                return false;
        }
        if (source.IsValidReverseSource())
        {
            if (source.reverse_segment_id.id >= number_of_nodes ||
                !insertSource(source_labels,
                              source.reverse_segment_id.id,
                              source.GetReverseWeightAsSource(),
                              source.GetReverseDurationAsSource()))
                return false;
        }
    }

    // A phantom source is a virtual point inside a directed geometry, rather than an ordinary
    // network node.  Do not settle its negative seed in the heap: a loop can return to the same
    // node with a real, positive network label.  Seed the upward CH arcs directly instead.
    for (const auto source : util::irange<NodeID>(0, number_of_nodes))
    {
        const auto &source_label = source_labels[source];
        if (!source_label.reachable)
            continue;

        for (const auto edge : facade.GetAdjacentEdgeRange(source))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!data.forward)
                continue;

            BOOST_ASSERT(data.weight > EdgeWeight{0});
            BOOST_ASSERT(to_alias<EdgeDuration>(data.duration) >= EdgeDuration{0});

            EdgeWeight weight;
            EdgeDuration duration;
            if (!checkedAdd(source_label.weight, data.weight, weight) ||
                !checkedAdd(source_label.duration, to_alias<EdgeDuration>(data.duration), duration))
                return false;

            insertOrUpdate(heap, facade.GetTarget(edge), weight, duration);
        }

        if (!relaxCoreBackwardEdges(heap, source_label, core_backward_edges[source]))
            return false;
    }
    return true;
}

template <typename CHFacade>
bool runUpwardSearch(const CHFacade &facade,
                     IsochroneCHQueryHeap &heap,
                     const CoreBackwardEdges &core_backward_edges)
{
    while (!heap.Empty())
    {
        // Insertions can reallocate the heap's backing storage, so retain the settled label by
        // value rather than holding the reference returned by DeleteMinGetHeapNode.
        const auto current = heap.DeleteMinGetHeapNode();
        for (const auto edge : facade.GetAdjacentEdgeRange(current.node))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!data.forward)
                continue;

            BOOST_ASSERT(data.weight > EdgeWeight{0});
            BOOST_ASSERT(to_alias<EdgeDuration>(data.duration) >= EdgeDuration{0});

            EdgeWeight weight;
            EdgeDuration duration;
            if (!checkedAdd(current.weight, data.weight, weight) ||
                !checkedAdd(current.data.duration, to_alias<EdgeDuration>(data.duration), duration))
                return false;

            insertOrUpdate(heap, facade.GetTarget(edge), weight, duration);
        }

        const IsochroneCHNodeLabel current_label{current.weight, current.data.duration, true};
        if (!relaxCoreBackwardEdges(heap, current_label, core_backward_edges[current.node]))
            return false;
    }
    return true;
}

inline std::vector<IsochroneCHNodeLabel> collectUpwardLabels(const IsochroneCHQueryHeap &heap,
                                                             const unsigned number_of_nodes)
{
    std::vector<IsochroneCHNodeLabel> labels(number_of_nodes);
    for (const auto node : util::irange<NodeID>(0, number_of_nodes))
    {
        if (heap.WasInserted(node))
            labels[node] = {heap.GetKey(node), heap.GetData(node).duration, true};
    }
    return labels;
}

template <typename CHFacade>
bool runDownwardSweep(const CHFacade &facade,
                      const std::vector<NodeID> &order,
                      const std::vector<IsochroneCHNodeLabel> &source_labels,
                      std::vector<IsochroneCHNodeLabel> &labels)
{
    std::vector<unsigned> rank(labels.size(), order.size());
    for (unsigned position = 0; position < order.size(); ++position)
        rank[order[position]] = position;

    for (auto iterator = order.rbegin(); iterator != order.rend(); ++iterator)
    {
        const auto lower = *iterator;
        for (const auto edge : facade.GetAdjacentEdgeRange(lower))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!data.backward)
                continue;

            const auto higher = facade.GetTarget(edge);
            BOOST_ASSERT(higher == lower || rank[lower] < rank[higher]);
            if (source_labels[higher].reachable &&
                !relaxLabel(source_labels[higher],
                            data.weight,
                            to_alias<EdgeDuration>(data.duration),
                            labels[lower]))
                return false;
            if (labels[higher].reachable && !relaxLabel(labels[higher],
                                                        data.weight,
                                                        to_alias<EdgeDuration>(data.duration),
                                                        labels[lower]))
                return false;
        }
    }
    return true;
}

} // namespace detail

// Runs an outbound, duration-bounded PHAST query over a fully contracted CH graph.  CHFacade
// needs GetNumberOfNodes, GetAdjacentEdgeRange, GetEdgeData, and GetTarget, which lets this
// kernel stay independent of facade ownership and request dispatch.
//
// Source seeds are virtual and are not returned as network labels; a later geometry materializer
// describes the source partial.  Labels are selected by profile weight and retain the duration of
// that selected CH path.  The cutoff filters final labels only, because a lower-weight label above
// the duration cutoff can still suppress a higher-weight, shorter-duration candidate downstream.
template <typename CHFacade>
IsochroneSearchResult phastOneToAllSearch(const CHFacade &facade,
                                          const PhantomNodeCandidates &source_candidates,
                                          const EdgeDuration duration_cutoff)
{
    IsochroneSearchResult result;
    BOOST_ASSERT(duration_cutoff >= EdgeDuration{0});

    std::vector<NodeID> order;
    detail::CoreBackwardEdges core_backward_edges;
    if (!detail::buildDownwardOrder(facade, order, core_backward_edges))
    {
        result.status = IsochroneSearchStatus::UnsupportedCHGraph;
        return result;
    }

    detail::IsochroneCHQueryHeap heap(facade.GetNumberOfNodes());
    std::vector<detail::IsochroneCHNodeLabel> source_labels(facade.GetNumberOfNodes());
    if (!detail::initializeSources(
            heap, source_labels, source_candidates, facade, core_backward_edges) ||
        !detail::runUpwardSearch(facade, heap, core_backward_edges))
    {
        result.status = IsochroneSearchStatus::ArithmeticOverflow;
        return result;
    }

    auto labels = detail::collectUpwardLabels(heap, facade.GetNumberOfNodes());
    if (!detail::runDownwardSweep(facade, order, source_labels, labels))
    {
        result.status = IsochroneSearchStatus::ArithmeticOverflow;
        return result;
    }

    result.nodes.reserve(labels.size());
    for (const auto node : util::irange<NodeID>(0, facade.GetNumberOfNodes()))
    {
        const auto &label = labels[node];
        if (label.reachable && label.duration <= duration_cutoff)
            result.nodes.push_back({node, label.weight, label.duration});
    }
    return result;
}

} // namespace osrm::engine::routing_algorithms::ch

#endif // OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP
