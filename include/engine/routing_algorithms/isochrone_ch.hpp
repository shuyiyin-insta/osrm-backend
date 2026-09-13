#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP

#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/isochrone_ch_topological_order.hpp"

#include "util/integer_range.hpp"
#include "util/query_heap.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <cstdint>
#include <limits>
#include <tuple>
#include <vector>

namespace osrm::engine::routing_algorithms::ch
{

enum class IsochroneSearchStatus : std::uint8_t
{
    Complete,
    // The filtered CH graph has an invalid edge endpoint and cannot be searched.
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
                       const CHFacade &facade)
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
    }
    return true;
}

template <typename CHFacade>
bool runUpwardSearch(const CHFacade &facade, IsochroneCHQueryHeap &heap)
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

inline bool relaxCoreLabel(const IsochroneCHNodeLabel &from,
                           const EdgeWeight edge_weight,
                           const EdgeDuration edge_duration,
                           const NodeID to,
                           std::vector<IsochroneCHNodeLabel> &labels,
                           IsochroneCHQueryHeap &heap)
{
    BOOST_ASSERT(from.reachable);
    BOOST_ASSERT(edge_weight > EdgeWeight{0});
    BOOST_ASSERT(edge_duration >= EdgeDuration{0});

    EdgeWeight weight;
    EdgeDuration duration;
    if (!checkedAdd(from.weight, edge_weight, weight) ||
        !checkedAdd(from.duration, edge_duration, duration))
        return false;

    if (isBetterLabel(weight, duration, labels[to]))
    {
        labels[to] = {weight, duration, true};
        insertOrUpdate(heap, to, weight, duration);
    }
    return true;
}

template <typename CHFacade>
bool relaxCoreOutgoingEdges(const CHFacade &facade,
                            const IsochroneCHTopologicalOrder &order,
                            const NodeID source,
                            const IsochroneCHNodeLabel &source_label,
                            std::vector<IsochroneCHNodeLabel> &labels,
                            IsochroneCHQueryHeap &heap)
{
    BOOST_ASSERT(order.isCoreNode(source));

    for (const auto edge : facade.GetAdjacentEdgeRange(source))
    {
        const auto &data = facade.GetEdgeData(edge);
        const auto target = facade.GetTarget(edge);
        if (data.forward && order.isCoreNode(target) &&
            !relaxCoreLabel(source_label,
                            data.weight,
                            to_alias<EdgeDuration>(data.duration),
                            target,
                            labels,
                            heap))
            return false;
    }

    const auto core_index = order.coreIndex(source);
    for (auto index = order.core_backward_edge_offsets[core_index];
         index < order.core_backward_edge_offsets[core_index + 1];
         ++index)
    {
        const auto &reverse_edge = order.core_backward_edges[index];
        if (!relaxCoreLabel(source_label,
                            reverse_edge.weight,
                            reverse_edge.duration,
                            reverse_edge.source,
                            labels,
                            heap))
            return false;
    }
    return true;
}

template <typename CHFacade>
bool runCoreSearch(const CHFacade &facade,
                   const IsochroneCHTopologicalOrder &order,
                   const std::vector<IsochroneCHNodeLabel> &source_labels,
                   std::vector<IsochroneCHNodeLabel> &labels)
{
    IsochroneCHQueryHeap heap(facade.GetNumberOfNodes());
    for (const auto node : util::irange<NodeID>(0, facade.GetNumberOfNodes()))
    {
        if (order.isCoreNode(node) && labels[node].reachable)
            heap.Insert(node, labels[node].weight, {labels[node].duration});
    }

    // A phantom source is virtual and therefore must not become an output label just by seeding
    // the core search.  Expand it directly; a genuine loop can still later create its label.
    for (const auto node : util::irange<NodeID>(0, facade.GetNumberOfNodes()))
    {
        if (order.isCoreNode(node) && source_labels[node].reachable &&
            !relaxCoreOutgoingEdges(
                facade, order, node, source_labels[node], labels, heap))
            return false;
    }

    while (!heap.Empty())
    {
        const auto current = heap.DeleteMinGetHeapNode();
        const IsochroneCHNodeLabel current_label{
            current.weight, current.data.duration, true};
        if (!relaxCoreOutgoingEdges(
                facade, order, current.node, current_label, labels, heap))
            return false;
    }
    return true;
}

template <typename CHFacade>
bool runDownwardSweep(const CHFacade &facade,
                      const IsochroneCHTopologicalOrder &order,
                      const std::vector<IsochroneCHNodeLabel> &source_labels,
                      std::vector<IsochroneCHNodeLabel> &labels)
{
    for (auto iterator = order.nodes.rbegin(); iterator != order.nodes.rend(); ++iterator)
    {
        const auto lower = *iterator;
        for (const auto edge : facade.GetAdjacentEdgeRange(lower))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!data.backward)
                continue;

            const auto higher = facade.GetTarget(edge);
            BOOST_ASSERT(higher == lower || order.rank[lower] < order.rank[higher]);
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

// Runs an outbound, duration-bounded PHAST query over a CH graph.  CHFacade needs
// GetIsochroneTopologicalOrder in addition to the graph accessors used below.  Keeping the order
// with the loaded facade prevents an O(nodes + edges) DAG walk for every request.
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

    const auto *order = facade.GetIsochroneTopologicalOrder();
    if (order == nullptr)
    {
        result.status = IsochroneSearchStatus::UnsupportedCHGraph;
        return result;
    }

    detail::IsochroneCHQueryHeap heap(facade.GetNumberOfNodes());
    std::vector<detail::IsochroneCHNodeLabel> source_labels(facade.GetNumberOfNodes());
    if (!detail::initializeSources(heap, source_labels, source_candidates, facade) ||
        !detail::runUpwardSearch(facade, heap))
    {
        result.status = IsochroneSearchStatus::ArithmeticOverflow;
        return result;
    }

    auto labels = detail::collectUpwardLabels(heap, facade.GetNumberOfNodes());
    if (!detail::runCoreSearch(facade, *order, source_labels, labels) ||
        !detail::runDownwardSweep(facade, *order, source_labels, labels))
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
