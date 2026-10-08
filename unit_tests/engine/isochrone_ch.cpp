#include "engine/routing_algorithms/isochrone_ch.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <mutex>
#include <optional>
#include <span>
#include <utility>
#include <vector>

namespace osrm::engine::routing_algorithms::ch
{
namespace
{

struct Edge
{
    NodeID source;
    NodeID target;
    EdgeWeight weight;
    EdgeDuration duration;
    bool forward;
    bool backward;
};

class SyntheticCHFacade
{
  public:
    SyntheticCHFacade(const unsigned number_of_nodes, const std::initializer_list<Edge> input_edges)
        : m_number_of_nodes(number_of_nodes), m_edges(input_edges)
    {
        std::sort(m_edges.begin(),
                  m_edges.end(),
                  [](const Edge &lhs, const Edge &rhs)
                  { return std::tie(lhs.source, lhs.target) < std::tie(rhs.source, rhs.target); });
    }

    unsigned GetNumberOfNodes() const { return m_number_of_nodes; }
    NodeID GetTarget(const Edge &edge) const { return edge.target; }
    const Edge &GetEdgeData(const Edge &edge) const { return edge; }
    std::span<const Edge> GetAdjacentEdgeRange(const NodeID node) const
    {
        const auto first = std::lower_bound(m_edges.begin(),
                                            m_edges.end(),
                                            node,
                                            [](const Edge &edge, const NodeID source)
                                            { return edge.source < source; });
        const auto last = std::upper_bound(first,
                                           m_edges.end(),
                                           node,
                                           [](const NodeID source, const Edge &edge)
                                           { return source < edge.source; });
        if (first == last)
            return {};
        return {&*first, static_cast<std::size_t>(last - first)};
    }

    const IsochroneCHTopologicalOrder *GetIsochroneTopologicalOrder() const
    {
        std::call_once(m_isochrone_topological_order_once,
                       [this]
                       {
                           ++m_isochrone_topological_order_build_count;
                           IsochroneCHTopologicalOrder order;
                           if (buildIsochroneCHTopologicalOrder(*this, order))
                               m_isochrone_topological_order = std::move(order);
                       });

        return m_isochrone_topological_order ? &*m_isochrone_topological_order : nullptr;
    }

    unsigned GetIsochroneTopologicalOrderBuildCount() const
    { return m_isochrone_topological_order_build_count; }

  private:
    unsigned m_number_of_nodes;
    std::vector<Edge> m_edges;
    mutable std::once_flag m_isochrone_topological_order_once;
    mutable std::optional<IsochroneCHTopologicalOrder> m_isochrone_topological_order;
    mutable unsigned m_isochrone_topological_order_build_count = 0;
};

PhantomNode makeSource(const NodeID node,
                       const EdgeWeight weight,
                       const EdgeWeight weight_offset,
                       const EdgeDuration duration,
                       const EdgeDuration duration_offset)
{
    struct Segment
    {
        SegmentID forward_segment_id;
        SegmentID reverse_segment_id;
        unsigned short fwd_segment_position;
    } segment{{node, true}, {SPECIAL_SEGMENTID, false}, 0};

    const util::Coordinate coordinate{util::FixedLongitude{0}, util::FixedLatitude{0}};
    return PhantomNode{segment,
                       ComponentID{1, false},
                       weight,
                       INVALID_EDGE_WEIGHT,
                       weight_offset,
                       EdgeWeight{0},
                       EdgeDistance{0},
                       INVALID_EDGE_DISTANCE,
                       EdgeDistance{0},
                       EdgeDistance{0},
                       duration,
                       MAXIMAL_EDGE_DURATION,
                       duration_offset,
                       EdgeDuration{0},
                       true,
                       false,
                       false,
                       false,
                       coordinate,
                       coordinate,
                       0};
}

PhantomNode makeReverseSource(const NodeID node,
                              const EdgeWeight weight,
                              const EdgeWeight weight_offset,
                              const EdgeDuration duration,
                              const EdgeDuration duration_offset)
{
    struct Segment
    {
        SegmentID forward_segment_id;
        SegmentID reverse_segment_id;
        unsigned short fwd_segment_position;
    } segment{{SPECIAL_SEGMENTID, false}, {node, true}, 0};

    const util::Coordinate coordinate{util::FixedLongitude{0}, util::FixedLatitude{0}};
    return PhantomNode{segment,
                       ComponentID{1, false},
                       INVALID_EDGE_WEIGHT,
                       weight,
                       EdgeWeight{0},
                       weight_offset,
                       INVALID_EDGE_DISTANCE,
                       EdgeDistance{0},
                       EdgeDistance{0},
                       EdgeDistance{0},
                       MAXIMAL_EDGE_DURATION,
                       duration,
                       EdgeDuration{0},
                       duration_offset,
                       false,
                       false,
                       true,
                       false,
                       coordinate,
                       coordinate,
                       0};
}

PhantomNode makeTarget(const NodeID node,
                       const EdgeWeight weight,
                       const EdgeWeight weight_offset,
                       const EdgeDuration duration,
                       const EdgeDuration duration_offset)
{
    struct Segment
    {
        SegmentID forward_segment_id;
        SegmentID reverse_segment_id;
        unsigned short fwd_segment_position;
    } segment{{node, true}, {SPECIAL_SEGMENTID, false}, 0};

    const util::Coordinate coordinate{util::FixedLongitude{0}, util::FixedLatitude{0}};
    return PhantomNode{segment,
                       ComponentID{1, false},
                       weight,
                       INVALID_EDGE_WEIGHT,
                       weight_offset,
                       EdgeWeight{0},
                       EdgeDistance{0},
                       INVALID_EDGE_DISTANCE,
                       EdgeDistance{0},
                       EdgeDistance{0},
                       duration,
                       MAXIMAL_EDGE_DURATION,
                       duration_offset,
                       EdgeDuration{0},
                       false,
                       true,
                       false,
                       false,
                       coordinate,
                       coordinate,
                       0};
}

const IsochroneSearchNode *findNode(const IsochroneSearchResult &result, const NodeID node)
{
    const auto found =
        std::find_if(result.nodes.begin(),
                     result.nodes.end(),
                     [node](const auto &candidate) { return candidate.node == node; });
    return found == result.nodes.end() ? nullptr : &*found;
}

} // namespace

BOOST_AUTO_TEST_SUITE(isochrone_ch)

BOOST_AUTO_TEST_CASE(outbound_phast_sweeps_downward_over_backward_ch_arcs)
{
    // Node 2 has higher CH rank than nodes 0 and 1.  The stored backward arc at node 1 represents
    // the outbound downward edge 2 -> 1.
    const SyntheticCHFacade facade{3,
                                   {{0, 2, EdgeWeight{10}, EdgeDuration{100}, true, false},
                                    {1, 2, EdgeWeight{20}, EdgeDuration{50}, false, true}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {150});

    BOOST_REQUIRE(result.isComplete());
    const auto *high = findNode(result, 2);
    const auto *downward = findNode(result, 1);
    BOOST_REQUIRE(high != nullptr);
    BOOST_REQUIRE(downward != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(high->weight), 10);
    BOOST_CHECK_EQUAL(from_alias<int>(high->duration), 100);
    BOOST_CHECK_EQUAL(from_alias<int>(downward->weight), 30);
    BOOST_CHECK_EQUAL(from_alias<int>(downward->duration), 150);
}

BOOST_AUTO_TEST_CASE(outbound_upward_search_does_not_follow_backward_ch_arcs)
{
    const SyntheticCHFacade facade{2, {{0, 1, EdgeWeight{10}, EdgeDuration{10}, false, true}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {100});

    BOOST_REQUIRE(result.isComplete());
    BOOST_CHECK(findNode(result, 1) == nullptr);
}

BOOST_AUTO_TEST_CASE(inbound_phast_swaps_the_upward_and_downward_ch_arcs)
{
    // In the original graph the path is 1 -> 2 -> 0.  Reverse search from target 0 first follows
    // the stored backward arc upward, then the stored forward arc downward.
    const SyntheticCHFacade facade{3,
                                   {{0, 2, EdgeWeight{10}, EdgeDuration{100}, false, true},
                                    {1, 2, EdgeWeight{20}, EdgeDuration{50}, true, false}}};

    const auto result = phastOneToAllSearch<false>(
        facade, {makeTarget(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {150});

    BOOST_REQUIRE(result.isComplete());
    const auto *high = findNode(result, 2);
    const auto *downward = findNode(result, 1);
    BOOST_REQUIRE(high != nullptr);
    BOOST_REQUIRE(downward != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(high->weight), 10);
    BOOST_CHECK_EQUAL(from_alias<int>(high->duration), 100);
    BOOST_CHECK_EQUAL(from_alias<int>(downward->weight), 30);
    BOOST_CHECK_EQUAL(from_alias<int>(downward->duration), 150);
}

BOOST_AUTO_TEST_CASE(inbound_search_does_not_follow_forward_arcs_upward)
{
    const SyntheticCHFacade facade{2, {{0, 1, EdgeWeight{10}, EdgeDuration{10}, true, false}}};

    const auto result = phastOneToAllSearch<false>(
        facade, {makeTarget(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {100});

    BOOST_REQUIRE(result.isComplete());
    BOOST_CHECK(findNode(result, 1) == nullptr);
}

BOOST_AUTO_TEST_CASE(reuses_the_topological_order_once_per_facade)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 2, EdgeWeight{10}, EdgeDuration{100}, true, false},
                                    {1, 2, EdgeWeight{20}, EdgeDuration{50}, false, true}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto first = phastOneToAllSearch(facade, {source}, {150});
    const auto second = phastOneToAllSearch(facade, {source}, {150});

    BOOST_REQUIRE(first.isComplete());
    BOOST_REQUIRE(second.isComplete());
    BOOST_CHECK_EQUAL(facade.GetIsochroneTopologicalOrderBuildCount(), 1);

    const SyntheticCHFacade replacement_facade{
        3,
        {{0, 2, EdgeWeight{10}, EdgeDuration{100}, true, false},
         {1, 2, EdgeWeight{20}, EdgeDuration{50}, false, true}}};
    const auto replacement_result = phastOneToAllSearch(replacement_facade, {source}, {150});

    BOOST_REQUIRE(replacement_result.isComplete());
    BOOST_CHECK_EQUAL(replacement_facade.GetIsochroneTopologicalOrderBuildCount(), 1);
}

BOOST_AUTO_TEST_CASE(virtual_source_seed_allows_forward_self_loop_reentry)
{
    // The negative phantom seed is virtual.  The loop returns to the same edge-based node with a
    // normal network label, so it must not be suppressed by the already settled source seed.
    const SyntheticCHFacade facade{1, {{0, 0, EdgeWeight{130}, EdgeDuration{130}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{80}, EdgeWeight{0}, {80}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, {60});

    BOOST_REQUIRE(result.isComplete());
    const auto *reentry = findNode(result, 0);
    BOOST_REQUIRE(reentry != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(reentry->weight), 50);
    BOOST_CHECK_EQUAL(from_alias<int>(reentry->duration), 50);
}

BOOST_AUTO_TEST_CASE(virtual_source_seed_allows_downward_self_loop_reentry)
{
    // A backward CH arc is traversed by the PHAST downward sweep.  Self-loops still represent a
    // real network re-entry and must not be skipped merely because both ranked endpoints match.
    const SyntheticCHFacade facade{1, {{0, 0, EdgeWeight{130}, EdgeDuration{130}, false, true}}};
    const auto source = makeSource(0, EdgeWeight{80}, EdgeWeight{0}, {80}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, {60});

    BOOST_REQUIRE(result.isComplete());
    const auto *reentry = findNode(result, 0);
    BOOST_REQUIRE(reentry != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(reentry->weight), 50);
    BOOST_CHECK_EQUAL(from_alias<int>(reentry->duration), 50);
}

BOOST_AUTO_TEST_CASE(uses_weight_optimal_path_and_carries_its_duration)
{
    // The route through node 1 is quicker but has a higher configured profile weight.  The CH
    // search must select the lower-weight route through node 2 and expose that route's duration.
    const SyntheticCHFacade facade{4,
                                   {{0, 1, EdgeWeight{1}, EdgeDuration{1}, true, false},
                                    {1, 3, EdgeWeight{10}, EdgeDuration{1}, true, false},
                                    {0, 2, EdgeWeight{2}, EdgeDuration{6}, true, false},
                                    {2, 3, EdgeWeight{2}, EdgeDuration{6}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto below_selected_duration = phastOneToAllSearch(facade, {source}, {10});
    BOOST_REQUIRE(below_selected_duration.isComplete());
    BOOST_CHECK(findNode(below_selected_duration, 3) == nullptr);

    const auto at_selected_duration = phastOneToAllSearch(facade, {source}, {12});
    BOOST_REQUIRE(at_selected_duration.isComplete());
    const auto *target = findNode(at_selected_duration, 3);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(target->weight), 4);
    BOOST_CHECK_EQUAL(from_alias<int>(target->duration), 12);
}

BOOST_AUTO_TEST_CASE(over_cutoff_weight_optimal_labels_still_suppress_faster_paths)
{
    // Node 2's selected label is already over the duration cutoff.  It still has to propagate to
    // node 3: the route through node 2 wins by profile weight, so node 3 cannot be emitted using
    // the faster but much higher-weight direct route.
    const SyntheticCHFacade facade{4,
                                   {{0, 2, EdgeWeight{1}, EdgeDuration{20}, true, false},
                                    {2, 3, EdgeWeight{1}, EdgeDuration{0}, true, false},
                                    {0, 3, EdgeWeight{100}, EdgeDuration{1}, true, false}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {10});

    BOOST_REQUIRE(result.isComplete());
    BOOST_CHECK(findNode(result, 3) == nullptr);
}

BOOST_AUTO_TEST_CASE(ignores_dominated_upward_weight_overflow)
{
    const SyntheticCHFacade facade{
        3,
        {{0, 1, EdgeWeight{1'200'000'000}, EdgeDuration{10}, true, false},
         {0, 2, EdgeWeight{1}, EdgeDuration{10}, true, false},
         {1, 2, EdgeWeight{1'200'000'000}, EdgeDuration{10}, true, false}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {10});

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 2);
    const auto *target = findNode(result, 2);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(target->weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(target->duration, EdgeDuration{10});
}

BOOST_AUTO_TEST_CASE(ignores_dominated_upward_duration_overflow_before_target_settlement)
{
    const SyntheticCHFacade facade{
        7,
        {{0, 1, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
         {0, 6, EdgeWeight{100}, EdgeDuration{0}, true, false},
         {1, 2, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
         {2, 3, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
         {3, 4, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
         {4, 5, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
         {5, 6, EdgeWeight{200}, EdgeDuration{400'000'000}, true, false}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {0});

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1);
    BOOST_CHECK_EQUAL(result.competitors.size(), 5);
    const auto *target = findNode(result, 6);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(target->weight, EdgeWeight{100});
    BOOST_CHECK_EQUAL(target->duration, EdgeDuration{0});
}

BOOST_AUTO_TEST_CASE(ignores_dominated_downward_weight_overflow)
{
    const SyntheticCHFacade facade{
        3,
        {{0, 1, EdgeWeight{1}, EdgeDuration{10}, true, false},
         {0, 2, EdgeWeight{1'200'000'000}, EdgeDuration{10}, true, false},
         {1, 2, EdgeWeight{1'200'000'000}, EdgeDuration{10}, false, true}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {10});

    BOOST_REQUIRE(result.isComplete());
    const auto *target = findNode(result, 1);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(target->weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(target->duration, EdgeDuration{10});
}

BOOST_AUTO_TEST_CASE(ignores_dominated_core_weight_overflow)
{
    const SyntheticCHFacade facade{
        3,
        {{0, 1, EdgeWeight{1'200'000'000}, EdgeDuration{10}, true, false},
         {0, 2, EdgeWeight{1}, EdgeDuration{10}, true, false},
         {1, 2, EdgeWeight{1'200'000'000}, EdgeDuration{10}, false, true},
         {2, 1, EdgeWeight{1'200'000'000}, EdgeDuration{10}, false, true}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {10});

    BOOST_REQUIRE(result.isComplete());
    const auto *target = findNode(result, 2);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(target->weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(target->duration, EdgeDuration{10});
}

BOOST_AUTO_TEST_CASE(reports_unrepresentable_weight_labels)
{
    const SyntheticCHFacade facade{
        3,
        {{0, 1, EdgeWeight{1'200'000'000}, EdgeDuration{10}, true, false},
         {1, 2, EdgeWeight{1'200'000'000}, EdgeDuration{10}, true, false}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {10});

    BOOST_CHECK(result.status == IsochroneSearchStatus::ArithmeticOverflow);
}

BOOST_AUTO_TEST_CASE(reports_unrepresentable_duration_labels)
{
    const SyntheticCHFacade facade{7,
                                   {{0, 1, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
                                    {2, 3, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
                                    {3, 4, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
                                    {4, 5, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false},
                                    {5, 6, EdgeWeight{1}, EdgeDuration{400'000'000}, true, false}}};

    const auto result =
        phastOneToAllSearch(facade, {makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0})}, {0});

    BOOST_CHECK(result.status == IsochroneSearchStatus::ArithmeticOverflow);
}

BOOST_AUTO_TEST_CASE(applies_forward_phantom_offsets_to_weight_and_duration_seeds)
{
    const SyntheticCHFacade facade{2, {{0, 1, EdgeWeight{20}, EdgeDuration{30}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{7}, EdgeWeight{3}, {7}, {3});

    const auto before_completion = phastOneToAllSearch(facade, {source}, {19});
    BOOST_REQUIRE(before_completion.isComplete());
    BOOST_CHECK(findNode(before_completion, 1) == nullptr);

    const auto at_completion = phastOneToAllSearch(facade, {source}, {20});
    BOOST_REQUIRE(at_completion.isComplete());
    const auto *target = findNode(at_completion, 1);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(target->weight), 10);
    BOOST_CHECK_EQUAL(from_alias<int>(target->duration), 20);
}

BOOST_AUTO_TEST_CASE(applies_reverse_phantom_offsets_to_weight_and_duration_seeds)
{
    const SyntheticCHFacade facade{2, {{0, 1, EdgeWeight{20}, EdgeDuration{30}, true, false}}};
    const auto source = makeReverseSource(0, EdgeWeight{7}, EdgeWeight{3}, {7}, {3});

    const auto result = phastOneToAllSearch(facade, {source}, {20});

    BOOST_REQUIRE(result.isComplete());
    const auto *target = findNode(result, 1);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(target->weight), 10);
    BOOST_CHECK_EQUAL(from_alias<int>(target->duration), 20);
}

BOOST_AUTO_TEST_CASE(searches_an_uncontracted_core_before_sweeping_its_contracted_fringe)
{
    // Nodes 1 and 2 are the uncontracted core.  Backward arcs are physically stored in reverse,
    // so the core search needs its cached reverse adjacency to traverse 1 -> 2 -> 1.  The arc at
    // node 0 represents the downward edge 1 -> 0 that PHAST traverses after the core search.
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{5}, EdgeDuration{5}, false, true},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{1}, false, true},
                                    {2, 1, EdgeWeight{1}, EdgeDuration{1}, false, true}}};
    const auto source = makeSource(1, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, {10});
    const auto repeated_result = phastOneToAllSearch(facade, {source}, {10});

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE(repeated_result.isComplete());
    const auto *fringe = findNode(result, 0);
    BOOST_REQUIRE(fringe != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(fringe->weight), 5);
    BOOST_CHECK_EQUAL(from_alias<int>(fringe->duration), 5);
    const auto *core_node = findNode(result, 2);
    BOOST_REQUIRE(core_node != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(core_node->weight), 1);
    BOOST_CHECK_EQUAL(from_alias<int>(core_node->duration), 1);
    BOOST_CHECK_EQUAL(facade.GetIsochroneTopologicalOrderBuildCount(), 1);
}

BOOST_AUTO_TEST_CASE(inbound_searches_forward_arcs_in_reverse_through_an_uncontracted_core)
{
    // Original forward core arcs form 1 -> 2 -> 1.  An inbound search traverses their cached
    // reverse adjacency, then the stored forward fringe arc as a downward reverse edge.
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{5}, EdgeDuration{5}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{1}, true, false},
                                    {2, 1, EdgeWeight{1}, EdgeDuration{1}, true, false}}};
    const auto target = makeTarget(1, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch<false>(facade, {target}, {10});

    BOOST_REQUIRE(result.isComplete());
    const auto *fringe = findNode(result, 0);
    BOOST_REQUIRE(fringe != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(fringe->weight), 5);
    BOOST_CHECK_EQUAL(from_alias<int>(fringe->duration), 5);
    const auto *core_node = findNode(result, 2);
    BOOST_REQUIRE(core_node != nullptr);
    BOOST_CHECK_EQUAL(from_alias<int>(core_node->weight), 1);
    BOOST_CHECK_EQUAL(from_alias<int>(core_node->duration), 1);
}

BOOST_AUTO_TEST_CASE(rejects_an_invalid_ch_graph_once)
{
    const SyntheticCHFacade facade{1, {{0, 1, EdgeWeight{1}, EdgeDuration{1}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, {10});
    const auto repeated_result = phastOneToAllSearch(facade, {source}, {10});

    BOOST_CHECK(result.status == IsochroneSearchStatus::UnsupportedCHGraph);
    BOOST_CHECK(result.nodes.empty());
    BOOST_CHECK(repeated_result.status == IsochroneSearchStatus::UnsupportedCHGraph);
    BOOST_CHECK(repeated_result.nodes.empty());
    BOOST_CHECK_EQUAL(facade.GetIsochroneTopologicalOrderBuildCount(), 1);
}

BOOST_AUTO_TEST_CASE(bounds_discoveries_on_a_large_loaded_map)
{
    const SyntheticCHFacade facade{10'000'001,
                                   {{0, 1, EdgeWeight{1}, EdgeDuration{10}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{10}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, EdgeDuration{20}, 2);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 2);
    BOOST_CHECK(result.competitors.empty());
    const auto *first = findNode(result, 1);
    const auto *second = findNode(result, 2);
    BOOST_REQUIRE(first != nullptr);
    BOOST_REQUIRE(second != nullptr);
    BOOST_CHECK_EQUAL(first->weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(first->duration, EdgeDuration{10});
    BOOST_CHECK_EQUAL(second->weight, EdgeWeight{2});
    BOOST_CHECK_EQUAL(second->duration, EdgeDuration{20});

    const auto limited = phastOneToAllSearch(facade, {source}, EdgeDuration{20}, 1);
    BOOST_CHECK(limited.status == IsochroneSearchStatus::SearchNodeLimitReached);
    BOOST_CHECK(limited.nodes.empty());
    BOOST_CHECK(limited.competitors.empty());
    BOOST_CHECK_EQUAL(facade.GetIsochroneTopologicalOrderBuildCount(), 1);
}

BOOST_AUTO_TEST_CASE(counts_initial_source_fanout_before_heap_insertion)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{1}, EdgeDuration{1}, true, false},
                                    {0, 2, EdgeWeight{1}, EdgeDuration{1}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, EdgeDuration{10}, 1);

    BOOST_CHECK(result.status == IsochroneSearchStatus::SearchNodeLimitReached);
    BOOST_CHECK(result.nodes.empty());
    BOOST_CHECK(result.competitors.empty());
}

BOOST_AUTO_TEST_CASE(counts_over_duration_competitors)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{1}, EdgeDuration{10}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{10}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto limited = phastOneToAllSearch(facade, {source}, EdgeDuration{0}, 1);
    BOOST_CHECK(limited.status == IsochroneSearchStatus::SearchNodeLimitReached);

    const auto complete = phastOneToAllSearch(facade, {source}, EdgeDuration{0}, 2);
    BOOST_REQUIRE(complete.isComplete());
    BOOST_CHECK(complete.nodes.empty());
    BOOST_CHECK_EQUAL(complete.competitors.size(), 2);
}

BOOST_AUTO_TEST_CASE(does_not_count_duplicate_paths_or_label_improvements_twice)
{
    const SyntheticCHFacade facade{4,
                                   {{0, 1, EdgeWeight{1}, EdgeDuration{1}, true, false},
                                    {0, 1, EdgeWeight{2}, EdgeDuration{1}, true, false},
                                    {0, 2, EdgeWeight{2}, EdgeDuration{6}, true, false},
                                    {1, 3, EdgeWeight{10}, EdgeDuration{1}, true, false},
                                    {2, 3, EdgeWeight{2}, EdgeDuration{6}, true, false}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source, source}, EdgeDuration{12}, 3);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 3);
    const auto *target = findNode(result, 3);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(target->weight, EdgeWeight{4});
    BOOST_CHECK_EQUAL(target->duration, EdgeDuration{12});
}

BOOST_AUTO_TEST_CASE(does_not_count_known_upward_labels_again_when_entering_the_core)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{5}, EdgeDuration{5}, true, false},
                                    {0, 2, EdgeWeight{10}, EdgeDuration{10}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{1}, false, true},
                                    {2, 1, EdgeWeight{1}, EdgeDuration{1}, false, true}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, EdgeDuration{10}, 2);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 2);
    const auto *target = findNode(result, 2);
    BOOST_REQUIRE(target != nullptr);
    BOOST_CHECK_EQUAL(target->weight, EdgeWeight{6});
    BOOST_CHECK_EQUAL(target->duration, EdgeDuration{6});
}

BOOST_AUTO_TEST_CASE(counts_core_and_downward_discoveries)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{5}, EdgeDuration{5}, false, true},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{1}, false, true},
                                    {2, 1, EdgeWeight{1}, EdgeDuration{1}, false, true}}};
    const auto source = makeSource(1, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    for (const auto limit : {1u, 2u})
    {
        const auto result = phastOneToAllSearch(facade, {source}, EdgeDuration{10}, limit);
        BOOST_CHECK(result.status == IsochroneSearchStatus::SearchNodeLimitReached);
        BOOST_CHECK(result.nodes.empty());
        BOOST_CHECK(result.competitors.empty());
    }

    const auto complete = phastOneToAllSearch(facade, {source}, EdgeDuration{10}, 3);
    BOOST_REQUIRE(complete.isComplete());
    BOOST_CHECK_EQUAL(complete.nodes.size(), 3);
}

BOOST_AUTO_TEST_CASE(inbound_counts_core_and_downward_discoveries)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 1, EdgeWeight{5}, EdgeDuration{5}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{1}, true, false},
                                    {2, 1, EdgeWeight{1}, EdgeDuration{1}, true, false}}};
    const auto target = makeTarget(1, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    for (const auto limit : {1u, 2u})
    {
        const auto result = phastOneToAllSearch<false>(facade, {target}, EdgeDuration{10}, limit);
        BOOST_CHECK(result.status == IsochroneSearchStatus::SearchNodeLimitReached);
        BOOST_CHECK(result.nodes.empty());
        BOOST_CHECK(result.competitors.empty());
    }

    const auto complete = phastOneToAllSearch<false>(facade, {target}, EdgeDuration{10}, 3);
    BOOST_REQUIRE(complete.isComplete());
    BOOST_CHECK_EQUAL(complete.nodes.size(), 3);
}

BOOST_AUTO_TEST_CASE(counts_phantom_reentry_but_not_the_virtual_source)
{
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});
    const SyntheticCHFacade isolated{1, {}};
    const auto empty = phastOneToAllSearch(isolated, {source}, EdgeDuration{10}, 0);
    BOOST_REQUIRE(empty.isComplete());
    BOOST_CHECK(empty.nodes.empty());
    BOOST_CHECK(empty.competitors.empty());

    const SyntheticCHFacade loop{1, {{0, 0, EdgeWeight{1}, EdgeDuration{1}, true, true}}};
    const auto limited = phastOneToAllSearch(loop, {source}, EdgeDuration{10}, 0);
    BOOST_CHECK(limited.status == IsochroneSearchStatus::SearchNodeLimitReached);

    const auto complete = phastOneToAllSearch(loop, {source}, EdgeDuration{10}, 1);
    BOOST_REQUIRE(complete.isComplete());
    BOOST_REQUIRE_EQUAL(complete.nodes.size(), 1);
    BOOST_CHECK_EQUAL(complete.nodes.front().node, 0);
    BOOST_CHECK_EQUAL(complete.nodes.front().weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(complete.nodes.front().duration, EdgeDuration{1});
}

BOOST_AUTO_TEST_CASE(preserves_ascending_source_order_for_equal_weight_paths)
{
    const SyntheticCHFacade facade{3,
                                   {{0, 2, EdgeWeight{1}, EdgeDuration{10}, true, false},
                                    {1, 2, EdgeWeight{1}, EdgeDuration{20}, true, false}}};
    const auto first = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});
    const auto second = makeSource(1, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {second, first}, EdgeDuration{20}, 1);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1);
    BOOST_CHECK_EQUAL(result.nodes.front().node, 2);
    BOOST_CHECK_EQUAL(result.nodes.front().weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(result.nodes.front().duration, EdgeDuration{10});
}

BOOST_AUTO_TEST_CASE(preserves_arithmetic_overflow_status_in_the_core)
{
    const SyntheticCHFacade facade{
        3,
        {{0, 1, EdgeWeight{1'200'000'000}, EdgeDuration{1}, true, false},
         {1, 2, EdgeWeight{1'200'000'000}, EdgeDuration{1}, false, true},
         {2, 1, EdgeWeight{1'200'000'000}, EdgeDuration{1}, false, true}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, EdgeDuration{10}, 3);

    BOOST_CHECK(result.status == IsochroneSearchStatus::ArithmeticOverflow);
    BOOST_CHECK(result.nodes.empty());
    BOOST_CHECK(result.competitors.empty());
}

BOOST_AUTO_TEST_CASE(preserves_arithmetic_overflow_status_in_the_downward_sweep)
{
    const SyntheticCHFacade facade{
        3,
        {{0, 2, EdgeWeight{1'200'000'000}, EdgeDuration{1}, true, false},
         {1, 2, EdgeWeight{1'200'000'000}, EdgeDuration{1}, false, true}}};
    const auto source = makeSource(0, EdgeWeight{0}, EdgeWeight{0}, {0}, {0});

    const auto result = phastOneToAllSearch(facade, {source}, EdgeDuration{10}, 3);

    BOOST_CHECK(result.status == IsochroneSearchStatus::ArithmeticOverflow);
    BOOST_CHECK(result.nodes.empty());
    BOOST_CHECK(result.competitors.empty());
}

BOOST_AUTO_TEST_CASE(rejects_new_labels_before_mutating_heap_or_reachable_state)
{
    detail::IsochroneCHSearchBudget budget(0);
    const detail::IsochroneCHNodeLabel source{EdgeWeight{0}, EdgeDuration{0}, true};
    detail::IsochroneCHNodeLabel target;

    const auto label_status =
        detail::relaxLabel(source, EdgeWeight{1}, EdgeDuration{1}, target, budget);
    BOOST_CHECK(label_status == IsochroneSearchStatus::SearchNodeLimitReached);
    BOOST_CHECK(!target.reachable);
    BOOST_CHECK_EQUAL(budget.nodeCount(), 0);

    detail::IsochroneCHQueryHeap heap(2);
    const auto heap_status =
        detail::relaxHeapLabel(source, EdgeWeight{1}, EdgeDuration{1}, 1, heap, budget);
    BOOST_CHECK(heap_status == IsochroneSearchStatus::SearchNodeLimitReached);
    BOOST_CHECK_EQUAL(heap.Occupancy(), 0);
    BOOST_CHECK_EQUAL(budget.nodeCount(), 0);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace osrm::engine::routing_algorithms::ch
