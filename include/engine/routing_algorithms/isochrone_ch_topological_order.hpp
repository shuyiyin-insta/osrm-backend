#ifndef OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_TOPOLOGICAL_ORDER_HPP
#define OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_TOPOLOGICAL_ORDER_HPP

#include "util/integer_range.hpp"
#include "util/typedefs.hpp"

#include <boost/assert.hpp>

#include <queue>
#include <vector>

namespace osrm::engine::routing_algorithms::ch
{

struct IsochroneCHTopologicalOrder
{
    // Kahn-eliminated nodes, in ascending CH order.  A partially contracted CH leaves its core
    // out of this vector; ordinary forward Dijkstra covers it before the PHAST downward sweep.
    std::vector<NodeID> nodes;
    // The contracted fringe is ranked [0, nodes.size()).  The retained, uncontracted core is
    // ranked immediately after it, so rank - nodes.size() indexes core_backward_edge_offsets.
    std::vector<unsigned> rank;

    struct CoreBackwardEdge
    {
        NodeID source;
        EdgeWeight weight;
        EdgeDuration duration;
    };

    // A backward CH arc is physically stored as source -> target, but logically traversed as
    // target -> source.  These offsets index reverse adjacency for arcs whose endpoints are
    // both in the uncontracted core.
    std::vector<unsigned> core_backward_edge_offsets;
    std::vector<CoreBackwardEdge> core_backward_edges;

    bool isCoreNode(const NodeID node) const { return rank[node] >= nodes.size(); }

    unsigned coreIndex(const NodeID node) const
    {
        BOOST_ASSERT(isCoreNode(node));
        return rank[node] - nodes.size();
    }
};

template <typename CHFacade>
bool buildIsochroneCHTopologicalOrder(const CHFacade &facade, IsochroneCHTopologicalOrder &result)
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

    result.nodes.clear();
    result.nodes.reserve(number_of_nodes);
    while (!ready.empty())
    {
        const auto node = ready.front();
        ready.pop();
        result.nodes.push_back(node);

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

    // Cycles are the uncontracted core of a partially contracted CH.  The upward Dijkstra has
    // already searched its physical forward arcs, while the reverse PHAST sweep must visit only
    // its contracted fringe.
    result.rank.assign(number_of_nodes, number_of_nodes);
    for (unsigned position = 0; position < result.nodes.size(); ++position)
        result.rank[result.nodes[position]] = position;

    const auto first_core_rank = static_cast<unsigned>(result.nodes.size());
    unsigned number_of_core_nodes = 0;
    for (const auto node : util::irange<NodeID>(0, number_of_nodes))
    {
        if (result.rank[node] == number_of_nodes)
            result.rank[node] = first_core_rank + number_of_core_nodes++;
    }

    result.core_backward_edge_offsets.assign(number_of_core_nodes + 1, 0);
    for (const auto source : util::irange<NodeID>(0, number_of_nodes))
    {
        if (!result.isCoreNode(source))
            continue;

        for (const auto edge : facade.GetAdjacentEdgeRange(source))
        {
            const auto &data = facade.GetEdgeData(edge);
            const auto target = facade.GetTarget(edge);
            if (data.backward && result.isCoreNode(target))
                ++result.core_backward_edge_offsets[result.coreIndex(target) + 1];
        }
    }

    for (unsigned index = 1; index < result.core_backward_edge_offsets.size(); ++index)
        result.core_backward_edge_offsets[index] += result.core_backward_edge_offsets[index - 1];

    result.core_backward_edges.resize(result.core_backward_edge_offsets.back());
    auto next_core_backward_edge = result.core_backward_edge_offsets;
    for (const auto source : util::irange<NodeID>(0, number_of_nodes))
    {
        if (!result.isCoreNode(source))
            continue;

        for (const auto edge : facade.GetAdjacentEdgeRange(source))
        {
            const auto &data = facade.GetEdgeData(edge);
            const auto target = facade.GetTarget(edge);
            if (data.backward && result.isCoreNode(target))
            {
                const auto index = result.coreIndex(target);
                result.core_backward_edges[next_core_backward_edge[index]++] = {
                    source, data.weight, to_alias<EdgeDuration>(data.duration)};
            }
        }
    }
    return true;
}

} // namespace osrm::engine::routing_algorithms::ch

#endif // OSRM_ENGINE_ROUTING_ALGORITHMS_ISOCHRONE_CH_TOPOLOGICAL_ORDER_HPP
