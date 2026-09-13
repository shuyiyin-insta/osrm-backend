#include "engine/routing_algorithms/reachability.hpp"

#include <boost/test/unit_test.hpp>

#include <algorithm>
#include <array>

namespace osrm::engine::routing_algorithms::mld
{
namespace
{
class ReachabilityFacade final
{
  public:
    explicit ReachabilityFacade(const bool include_source_self_loop = false)
        : m_include_source_self_loop(include_source_self_loop)
    {
    }

    struct Partition
    {
        CellID GetCell(LevelID, NodeID) const { return 0; }
    };

    struct Metric
    {
    };

    struct Cell
    {
        auto GetDestinationNodes() const
        { return std::ranges::subrange(empty_nodes.begin(), empty_nodes.end()); }

        auto GetSourceNodes() const
        { return std::ranges::subrange(empty_nodes.begin(), empty_nodes.end()); }

        auto GetOutWeight(NodeID) const
        { return std::ranges::subrange(empty_weights.begin(), empty_weights.end()); }

        auto GetInWeight(NodeID) const
        { return std::ranges::subrange(empty_weights.begin(), empty_weights.end()); }

      private:
        inline static const std::array<NodeID, 0> empty_nodes{};
        inline static const std::array<EdgeWeight, 0> empty_weights{};
    };

    struct CellStorage
    {
        Cell GetCell(Metric, LevelID, CellID) const { return {}; }
    };

    unsigned GetNumberOfNodes() const { return 7; }
    unsigned GetMaxBorderNodeID() const { return 0; }

    const Partition &GetMultiLevelPartition() const { return partition; }
    const CellStorage &GetCellStorage() const { return cell_storage; }
    const Metric &GetCellMetric() const { return metric; }

    auto GetBorderEdgeRange(LevelID level, NodeID node) const
    {
        BOOST_ASSERT(level == 0);
        constexpr std::array<std::pair<EdgeID, EdgeID>, 7> edge_ranges = {
            {{0, 4}, {4, 6}, {6, 7}, {7, 7}, {7, 7}, {7, 7}, {7, 7}}};
        const auto [first, last] = edge_ranges[node];
        return util::irange<EdgeID>(first, last);
    }

    bool IsForwardEdge(EdgeID edge) const { return edge != 3 || m_include_source_self_loop; }
    bool IsBackwardEdge(EdgeID) const { return false; }
    bool ExcludeNode(NodeID node) const { return node == 4; }

    NodeID GetTarget(EdgeID edge) const
    {
        constexpr std::array<NodeID, 7> targets = {{1, 2, 5, 0, 3, 4, 3}};
        return targets[edge];
    }

    const customizer::EdgeBasedGraphEdgeData &GetEdgeData(EdgeID edge) const
    {
        static const std::array<customizer::EdgeBasedGraphEdgeData, 7> edge_data = {
            {{0}, {1}, {2}, {3}, {4}, {5}, {6}}};
        return edge_data[edge];
    }

    EdgeWeight GetNodeWeight(NodeID node) const
    {
        constexpr std::array<EdgeWeight, 7> weights = {{{201}, {1}, {1}, {1}, {1}, {100}, {1}}};
        return weights[node];
    }

    EdgeDuration GetNodeDuration(NodeID node) const
    {
        constexpr std::array<EdgeDuration, 7> durations = {{{13}, {1}, {1}, {1}, {1}, {1}, {1}}};
        return durations[node];
    }

    TurnPenalty GetWeightPenaltyForEdgeID(NodeID turn_id) const
    {
        constexpr std::array<TurnPenalty, 7> penalties = {
            {{0}, {-199}, {-100}, {0}, {0}, {0}, {0}}};
        return penalties[turn_id];
    }

    TurnPenalty GetDurationPenaltyForEdgeID(NodeID turn_id) const
    {
        constexpr std::array<TurnPenalty, 7> penalties = {{{1}, {98}, {0}, {0}, {0}, {0}, {0}}};
        return penalties[turn_id];
    }

  private:
    Partition partition;
    CellStorage cell_storage;
    Metric metric;
    bool m_include_source_self_loop;
};

PhantomNode makeSource()
{
    struct Seed
    {
        SegmentID forward_segment_id{0, true};
        SegmentID reverse_segment_id{SPECIAL_SEGMENTID, false};
        unsigned short fwd_segment_position = 0;
    };

    return PhantomNode{Seed{}, {1, false}, {5},  INVALID_EDGE_WEIGHT,
                       {95},   {0},        {0},  {0},
                       {0},    {0},        {4},  MAXIMAL_EDGE_DURATION,
                       {6},    {0},        true, false,
                       false,  false,      {},   {},
                       0};
}
} // namespace

BOOST_AUTO_TEST_SUITE(reachability)

BOOST_AUTO_TEST_CASE(over_duration_lower_weight_labels_suppress_in_cutoff_competitors)
{
    SearchEngineData<Algorithm> heaps;
    const ReachabilityFacade facade;

    const auto result = reachabilitySearch(heaps, facade, {makeSource()}, EdgeDuration{5});
    const auto &reachable_nodes = result.nodes;

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(reachable_nodes.size(), 2);

    BOOST_CHECK_EQUAL(reachable_nodes[0].node, 5);
    BOOST_CHECK_EQUAL(reachable_nodes[0].weight, EdgeWeight{1});
    BOOST_CHECK_EQUAL(reachable_nodes[0].duration, EdgeDuration{3});

    BOOST_CHECK_EQUAL(reachable_nodes[1].node, 1);
    BOOST_CHECK_EQUAL(reachable_nodes[1].weight, EdgeWeight{101});
    BOOST_CHECK_EQUAL(reachable_nodes[1].duration, EdgeDuration{4});

    // The direct route through node 1 reaches node 3 in five deciseconds with weight 102.  The
    // route through node 2 has lower weight (-97) but duration 102 and must suppress it.
    BOOST_CHECK(std::none_of(reachable_nodes.begin(),
                             reachable_nodes.end(),
                             [](const auto &result) { return result.node == 3; }));
}

BOOST_AUTO_TEST_CASE(charges_the_source_node_duration_and_turn_penalty)
{
    SearchEngineData<Algorithm> heaps;
    const ReachabilityFacade facade;

    const auto result = reachabilitySearch(heaps, facade, {makeSource()}, EdgeDuration{4});
    const auto &reachable_nodes = result.nodes;

    BOOST_REQUIRE(result.isComplete());
    const auto node_one = std::find_if(reachable_nodes.begin(),
                                       reachable_nodes.end(),
                                       [](const auto &result) { return result.node == 1; });
    BOOST_REQUIRE(node_one != reachable_nodes.end());
    // The source seed is -10.  Edge 0 charges node 0's duration (13) and its turn penalty (1).
    // Using the target node's duration would instead yield -8 and fail to model the segment.
    BOOST_CHECK_EQUAL(node_one->duration, EdgeDuration{4});
}

BOOST_AUTO_TEST_CASE(uses_every_source_candidate)
{
    SearchEngineData<Algorithm> heaps;
    const ReachabilityFacade facade;
    auto additional_source = makeSource();
    additional_source.forward_segment_id = {1, true};
    additional_source.forward_weight = {0};
    additional_source.forward_weight_offset = {200};
    additional_source.forward_duration = {0};
    additional_source.forward_duration_offset = {0};

    const auto result =
        reachabilitySearch(heaps, facade, {makeSource(), additional_source}, EdgeDuration{1});
    const auto &reachable_nodes = result.nodes;

    BOOST_REQUIRE(result.isComplete());
    const auto node_three = std::find_if(reachable_nodes.begin(),
                                         reachable_nodes.end(),
                                         [](const auto &result) { return result.node == 3; });
    BOOST_REQUIRE(node_three != reachable_nodes.end());
    BOOST_CHECK_EQUAL(node_three->weight, EdgeWeight{-199});
    BOOST_CHECK_EQUAL(node_three->duration, EdgeDuration{1});
}

BOOST_AUTO_TEST_CASE(allows_a_real_path_to_reenter_its_source_node)
{
    SearchEngineData<Algorithm> heaps;
    const ReachabilityFacade facade{true};

    const auto result = reachabilitySearch(heaps, facade, {makeSource()}, EdgeDuration{3});
    const auto &reachable_nodes = result.nodes;

    BOOST_REQUIRE(result.isComplete());
    const auto source_node = std::find_if(reachable_nodes.begin(),
                                          reachable_nodes.end(),
                                          [](const auto &result) { return result.node == 0; });
    BOOST_REQUIRE(source_node != reachable_nodes.end());
    BOOST_CHECK_EQUAL(source_node->weight, EdgeWeight{101});
    BOOST_CHECK_EQUAL(source_node->duration, EdgeDuration{3});
}

BOOST_AUTO_TEST_SUITE_END()
} // namespace osrm::engine::routing_algorithms::mld
