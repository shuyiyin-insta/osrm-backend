#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP

#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/isochrone_ch_topological_order.hpp"

#include "util/integer_range.hpp"
#include "util/query_heap.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <tuple>
#include <vector>

namespace osrm::engine::routing_algorithms::ch
{

enum class IsochroneSearchStatus : std::uint8_t
{
    Complete,
    SearchNodeLimitReached,
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
    std::vector<IsochroneSearchNode> competitors;
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

using IsochroneCHSourceLabels = std::map<NodeID, IsochroneCHNodeLabel>;

class IsochroneCHSearchBudget
{
  public:
    explicit IsochroneCHSearchBudget(const std::size_t maximum_nodes)
        : m_maximum_nodes(maximum_nodes)
    {
    }

    IsochroneSearchStatus tryAddNode()
    {
        if (m_node_count >= m_maximum_nodes)
            return IsochroneSearchStatus::SearchNodeLimitReached;

        ++m_node_count;
        return IsochroneSearchStatus::Complete;
    }

    std::size_t nodeCount() const { return m_node_count; }

  private:
    std::size_t m_maximum_nodes;
    std::size_t m_node_count = 0;
};

template <typename Metric> bool checkedMetric(const std::int64_t value, Metric &result)
{
    using Value = typename Metric::value_type;
    if (value < std::numeric_limits<Value>::min() || value >= std::numeric_limits<Value>::max())
        return false;

    result = Metric{static_cast<Value>(value)};
    return true;
}

template <typename Metric> bool isValidMetric(const Metric metric)
{
    using Value = typename Metric::value_type;
    return from_alias<Value>(metric) < std::numeric_limits<Value>::max();
}

inline bool isBetterLabel(const std::int64_t candidate_weight, const IsochroneCHNodeLabel &current)
{
    // CH preprocessing chooses paths by profile weight.  Its shortcuts can have discarded a
    // different equal-weight path with a shorter duration, so duration is carried metadata, not
    // a secondary shortest-path criterion.
    return !current.reachable || candidate_weight < from_alias<std::int64_t>(current.weight);
}

inline bool isBetterLabel(const std::int64_t candidate_weight,
                          const IsochroneCHQueryHeap::HeapNode &current)
{ return candidate_weight < from_alias<std::int64_t>(current.weight); }

inline IsochroneSearchStatus relaxLabel(const IsochroneCHNodeLabel &from,
                                        const EdgeWeight edge_weight,
                                        const EdgeDuration edge_duration,
                                        IsochroneCHNodeLabel &to,
                                        IsochroneCHSearchBudget &budget)
{
    BOOST_ASSERT(from.reachable);
    BOOST_ASSERT(edge_weight > EdgeWeight{0});
    BOOST_ASSERT(edge_duration >= EdgeDuration{0});

    const auto candidate_weight =
        from_alias<std::int64_t>(from.weight) + from_alias<std::int64_t>(edge_weight);
    if (!isBetterLabel(candidate_weight, to))
        return IsochroneSearchStatus::Complete;

    EdgeWeight weight;
    EdgeDuration duration;
    if (!checkedMetric(candidate_weight, weight) ||
        !checkedMetric(from_alias<std::int64_t>(from.duration) +
                           from_alias<std::int64_t>(edge_duration),
                       duration))
        return IsochroneSearchStatus::ArithmeticOverflow;

    if (!to.reachable)
    {
        const auto status = budget.tryAddNode();
        if (status != IsochroneSearchStatus::Complete)
            return status;
    }

    to = {weight, duration, true};
    return IsochroneSearchStatus::Complete;
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

    if (current->WasRemoved() || !isBetterLabel(from_alias<std::int64_t>(weight), *current))
        return;

    current->data.duration = duration;
    if (weight < current->weight)
    {
        current->weight = weight;
        heap.DecreaseKey(*current);
    }
}

inline IsochroneSearchStatus relaxHeapLabel(const IsochroneCHNodeLabel &from,
                                            const EdgeWeight edge_weight,
                                            const EdgeDuration edge_duration,
                                            const NodeID to,
                                            IsochroneCHQueryHeap &heap,
                                            IsochroneCHSearchBudget &budget)
{
    BOOST_ASSERT(from.reachable);
    BOOST_ASSERT(edge_weight > EdgeWeight{0});
    BOOST_ASSERT(edge_duration >= EdgeDuration{0});

    const auto candidate_weight =
        from_alias<std::int64_t>(from.weight) + from_alias<std::int64_t>(edge_weight);
    const auto current = heap.GetHeapNodeIfWasInserted(to);
    if (current && (current->WasRemoved() || !isBetterLabel(candidate_weight, *current)))
        return IsochroneSearchStatus::Complete;

    EdgeWeight weight;
    EdgeDuration duration;
    if (!checkedMetric(candidate_weight, weight) ||
        !checkedMetric(from_alias<std::int64_t>(from.duration) +
                           from_alias<std::int64_t>(edge_duration),
                       duration))
        return IsochroneSearchStatus::ArithmeticOverflow;

    if (!current)
    {
        const auto status = budget.tryAddNode();
        if (status != IsochroneSearchStatus::Complete)
            return status;
    }

    insertOrUpdate(heap, to, weight, duration);
    return IsochroneSearchStatus::Complete;
}

inline IsochroneSearchStatus insertSource(IsochroneCHSourceLabels &source_labels,
                                          const NodeID node,
                                          const EdgeWeight weight,
                                          const EdgeDuration duration)
{
    if (!isValidMetric(weight) || !isValidMetric(duration))
        return IsochroneSearchStatus::ArithmeticOverflow;

    auto &current = source_labels[node];
    if (isBetterLabel(from_alias<std::int64_t>(weight), current))
        current = {weight, duration, true};
    return IsochroneSearchStatus::Complete;
}

template <bool FORWARD_SEARCH, typename CHFacade>
IsochroneSearchStatus initializeSources(IsochroneCHQueryHeap &heap,
                                        IsochroneCHSourceLabels &source_labels,
                                        const PhantomNodeCandidates &endpoint_candidates,
                                        const CHFacade &facade,
                                        IsochroneCHSearchBudget &budget)
{
    const auto number_of_nodes = facade.GetNumberOfNodes();
    for (const auto &endpoint : endpoint_candidates)
    {
        const auto forward_is_valid =
            FORWARD_SEARCH ? endpoint.IsValidForwardSource() : endpoint.IsValidForwardTarget();
        if (forward_is_valid)
        {
            const auto weight = FORWARD_SEARCH ? endpoint.GetForwardWeightAsSource()
                                               : endpoint.GetForwardWeightAsTarget();
            const auto duration = FORWARD_SEARCH ? endpoint.GetForwardDurationAsSource()
                                                 : endpoint.GetForwardDurationAsTarget();
            if (endpoint.forward_segment_id.id >= number_of_nodes)
                return IsochroneSearchStatus::ArithmeticOverflow;
            const auto status =
                insertSource(source_labels, endpoint.forward_segment_id.id, weight, duration);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
        const auto reverse_is_valid =
            FORWARD_SEARCH ? endpoint.IsValidReverseSource() : endpoint.IsValidReverseTarget();
        if (reverse_is_valid)
        {
            const auto weight = FORWARD_SEARCH ? endpoint.GetReverseWeightAsSource()
                                               : endpoint.GetReverseWeightAsTarget();
            const auto duration = FORWARD_SEARCH ? endpoint.GetReverseDurationAsSource()
                                                 : endpoint.GetReverseDurationAsTarget();
            if (endpoint.reverse_segment_id.id >= number_of_nodes)
                return IsochroneSearchStatus::ArithmeticOverflow;
            const auto status =
                insertSource(source_labels, endpoint.reverse_segment_id.id, weight, duration);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
    }

    // A phantom source is a virtual point inside a directed geometry, rather than an ordinary
    // network node.  Do not settle its negative seed in the heap: a loop can return to the same
    // node with a real, positive network label.  Seed the upward CH arcs directly instead.
    for (const auto &[source, source_label] : source_labels)
    {
        for (const auto edge : facade.GetAdjacentEdgeRange(source))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!(FORWARD_SEARCH ? data.forward : data.backward))
                continue;

            const auto status = relaxHeapLabel(source_label,
                                               data.weight,
                                               to_alias<EdgeDuration>(data.duration),
                                               facade.GetTarget(edge),
                                               heap,
                                               budget);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
    }
    return IsochroneSearchStatus::Complete;
}

template <bool FORWARD_SEARCH, typename CHFacade>
IsochroneSearchStatus
runUpwardSearch(const CHFacade &facade, IsochroneCHQueryHeap &heap, IsochroneCHSearchBudget &budget)
{
    while (!heap.Empty())
    {
        // Insertions can reallocate the heap's backing storage, so retain the settled label by
        // value rather than holding the reference returned by DeleteMinGetHeapNode.
        const auto current = heap.DeleteMinGetHeapNode();
        const IsochroneCHNodeLabel current_label{current.weight, current.data.duration, true};
        for (const auto edge : facade.GetAdjacentEdgeRange(current.node))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!(FORWARD_SEARCH ? data.forward : data.backward))
                continue;

            const auto status = relaxHeapLabel(current_label,
                                               data.weight,
                                               to_alias<EdgeDuration>(data.duration),
                                               facade.GetTarget(edge),
                                               heap,
                                               budget);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
    }
    return IsochroneSearchStatus::Complete;
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

inline IsochroneSearchStatus relaxCoreLabel(const IsochroneCHNodeLabel &from,
                                            const EdgeWeight edge_weight,
                                            const EdgeDuration edge_duration,
                                            const NodeID to,
                                            std::vector<IsochroneCHNodeLabel> &labels,
                                            IsochroneCHQueryHeap &heap,
                                            IsochroneCHSearchBudget &budget)
{
    const auto previous_weight = labels[to].weight;
    const auto status = relaxLabel(from, edge_weight, edge_duration, labels[to], budget);
    if (status != IsochroneSearchStatus::Complete)
        return status;

    if (labels[to].weight != previous_weight)
        insertOrUpdate(heap, to, labels[to].weight, labels[to].duration);
    return IsochroneSearchStatus::Complete;
}

template <bool FORWARD_SEARCH, typename CHFacade>
IsochroneSearchStatus relaxCoreOutgoingEdges(const CHFacade &facade,
                                             const IsochroneCHTopologicalOrder &order,
                                             const NodeID source,
                                             const IsochroneCHNodeLabel &source_label,
                                             std::vector<IsochroneCHNodeLabel> &labels,
                                             IsochroneCHQueryHeap &heap,
                                             IsochroneCHSearchBudget &budget)
{
    BOOST_ASSERT(order.isCoreNode(source));

    for (const auto edge : facade.GetAdjacentEdgeRange(source))
    {
        const auto &data = facade.GetEdgeData(edge);
        const auto target = facade.GetTarget(edge);
        if ((FORWARD_SEARCH ? data.forward : data.backward) && order.isCoreNode(target))
        {
            const auto status = relaxCoreLabel(source_label,
                                               data.weight,
                                               to_alias<EdgeDuration>(data.duration),
                                               target,
                                               labels,
                                               heap,
                                               budget);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
    }

    const auto relax_reverse_edges = [&](const auto &offsets, const auto &edges)
    {
        const auto core_index = order.coreIndex(source);
        for (auto index = offsets[core_index]; index < offsets[core_index + 1]; ++index)
        {
            const auto &reverse_edge = edges[index];
            const auto status = relaxCoreLabel(source_label,
                                               reverse_edge.weight,
                                               reverse_edge.duration,
                                               reverse_edge.source,
                                               labels,
                                               heap,
                                               budget);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
        return IsochroneSearchStatus::Complete;
    };

    if constexpr (FORWARD_SEARCH)
        return relax_reverse_edges(order.core_backward_edge_offsets, order.core_backward_edges);
    else
        return relax_reverse_edges(order.core_forward_edge_offsets, order.core_forward_edges);
}

template <bool FORWARD_SEARCH, typename CHFacade>
IsochroneSearchStatus runCoreSearch(const CHFacade &facade,
                                    const IsochroneCHTopologicalOrder &order,
                                    const IsochroneCHSourceLabels &source_labels,
                                    std::vector<IsochroneCHNodeLabel> &labels,
                                    IsochroneCHSearchBudget &budget)
{
    IsochroneCHQueryHeap heap(facade.GetNumberOfNodes());
    for (const auto node : util::irange<NodeID>(0, facade.GetNumberOfNodes()))
    {
        if (order.isCoreNode(node) && labels[node].reachable)
            heap.Insert(node, labels[node].weight, {labels[node].duration});
    }

    // A phantom source is virtual and therefore must not become an output label just by seeding
    // the core search.  Expand it directly; a genuine loop can still later create its label.
    for (const auto &[node, source_label] : source_labels)
    {
        if (order.isCoreNode(node))
        {
            const auto status = relaxCoreOutgoingEdges<FORWARD_SEARCH>(
                facade, order, node, source_label, labels, heap, budget);
            if (status != IsochroneSearchStatus::Complete)
                return status;
        }
    }

    while (!heap.Empty())
    {
        const auto current = heap.DeleteMinGetHeapNode();
        const IsochroneCHNodeLabel current_label{current.weight, current.data.duration, true};
        const auto status = relaxCoreOutgoingEdges<FORWARD_SEARCH>(
            facade, order, current.node, current_label, labels, heap, budget);
        if (status != IsochroneSearchStatus::Complete)
            return status;
    }
    return IsochroneSearchStatus::Complete;
}

template <bool FORWARD_SEARCH, typename CHFacade>
IsochroneSearchStatus runDownwardSweep(const CHFacade &facade,
                                       const IsochroneCHTopologicalOrder &order,
                                       const IsochroneCHSourceLabels &source_labels,
                                       std::vector<IsochroneCHNodeLabel> &labels,
                                       IsochroneCHSearchBudget &budget)
{
    for (auto iterator = order.nodes.rbegin(); iterator != order.nodes.rend(); ++iterator)
    {
        const auto lower = *iterator;
        for (const auto edge : facade.GetAdjacentEdgeRange(lower))
        {
            const auto &data = facade.GetEdgeData(edge);
            if (!(FORWARD_SEARCH ? data.backward : data.forward))
                continue;

            const auto higher = facade.GetTarget(edge);
            BOOST_ASSERT(higher == lower || order.rank[lower] < order.rank[higher]);
            const auto source = source_labels.find(higher);
            if (source != source_labels.end())
            {
                const auto status = relaxLabel(source->second,
                                               data.weight,
                                               to_alias<EdgeDuration>(data.duration),
                                               labels[lower],
                                               budget);
                if (status != IsochroneSearchStatus::Complete)
                    return status;
            }
            if (labels[higher].reachable)
            {
                const auto status = relaxLabel(labels[higher],
                                               data.weight,
                                               to_alias<EdgeDuration>(data.duration),
                                               labels[lower],
                                               budget);
                if (status != IsochroneSearchStatus::Complete)
                    return status;
            }
        }
    }
    return IsochroneSearchStatus::Complete;
}

} // namespace detail

// Runs a weight-optimal one-to-all PHAST query over a CH graph.  CHFacade needs
// GetIsochroneTopologicalOrder in addition to the graph accessors used below.  Keeping the order
// with the loaded facade prevents an O(nodes + edges) DAG walk for every request.
//
// Source seeds are virtual and are not returned as network labels; a later geometry materializer
// describes the source partial.  Labels are selected by profile weight and retain the duration of
// that selected CH path.  The cutoff filters final labels only, because a lower-weight label above
// the duration cutoff can still suppress a higher-weight, shorter-duration candidate downstream.
// The node budget counts distinct discovered network labels across phases, not loaded map nodes.
template <bool FORWARD_SEARCH = true, typename CHFacade>
IsochroneSearchResult phastOneToAllSearch(
    const CHFacade &facade,
    const PhantomNodeCandidates &endpoint_candidates,
    const EdgeDuration duration_cutoff,
    const std::size_t maximum_search_nodes = std::numeric_limits<std::size_t>::max())
{
    IsochroneSearchResult result;
    BOOST_ASSERT(duration_cutoff >= EdgeDuration{0});

    const auto *order = facade.GetIsochroneTopologicalOrder();
    if (order == nullptr)
    {
        result.status = IsochroneSearchStatus::UnsupportedCHGraph;
        return result;
    }

    detail::IsochroneCHSearchBudget budget(maximum_search_nodes);
    detail::IsochroneCHSourceLabels source_labels;
    std::vector<detail::IsochroneCHNodeLabel> labels;
    {
        detail::IsochroneCHQueryHeap heap(facade.GetNumberOfNodes());
        result.status = detail::initializeSources<FORWARD_SEARCH>(
            heap, source_labels, endpoint_candidates, facade, budget);
        if (!result.isComplete())
            return result;

        result.status = detail::runUpwardSearch<FORWARD_SEARCH>(facade, heap, budget);
        if (!result.isComplete())
            return result;

        labels = detail::collectUpwardLabels(heap, facade.GetNumberOfNodes());
    }

    result.status =
        detail::runCoreSearch<FORWARD_SEARCH>(facade, *order, source_labels, labels, budget);
    if (!result.isComplete())
        return result;

    result.status =
        detail::runDownwardSweep<FORWARD_SEARCH>(facade, *order, source_labels, labels, budget);
    if (!result.isComplete())
        return result;

    result.nodes.reserve(budget.nodeCount());
    for (const auto node : util::irange<NodeID>(0, facade.GetNumberOfNodes()))
    {
        const auto &label = labels[node];
        if (label.reachable && label.duration <= duration_cutoff)
            result.nodes.push_back({node, label.weight, label.duration});
        else if (label.reachable)
            result.competitors.push_back({node, label.weight, label.duration});
    }
    return result;
}

} // namespace osrm::engine::routing_algorithms::ch

#endif // OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_HPP
