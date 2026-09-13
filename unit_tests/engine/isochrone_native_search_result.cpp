#include "engine/isochrone/native_search_result.hpp"

#include "mocks/mock_datafacade.hpp"

#include <boost/test/unit_test.hpp>

#include <array>
#include <cstdint>

namespace
{

class NativeResultFacade final : public osrm::test::MockBaseDataFacade
{
  public:
    NativeResultFacade()
        : durations(osrm::util::vector_view<std::uint64_t>(duration_storage.data(),
                                                           duration_storage.size()),
                    1)
    {
    }

    GeometryID GetGeometryIndex(const NodeID node) const override
    {
        if (node == 99)
            return {SPECIAL_GEOMETRYID, false};
        return {static_cast<PackedGeometryID>(node / 2), node % 2 == 0};
    }

    DurationForwardRange GetUncompressedForwardDurations(const PackedGeometryID) const override
    { return {durations.begin(), durations.end()}; }

    DurationReverseRange GetUncompressedReverseDurations(const PackedGeometryID) const override
    { return DurationReverseRange{DurationForwardRange{durations.begin(), durations.end()}}; }

  private:
    std::array<std::uint64_t, 2> duration_storage = {100, 0};
    osrm::extractor::SegmentDataView::SegmentDurationVector durations;
};

} // namespace

BOOST_AUTO_TEST_SUITE(isochrone_native_search_result)

BOOST_AUTO_TEST_CASE(retains_only_competitors_for_candidate_geometries)
{
    const NativeResultFacade facade;
    osrm::engine::routing_algorithms::ReachabilitySearchResult native_result;
    native_result.nodes.push_back({0, EdgeWeight{100}, EdgeDuration{50}});
    native_result.competitors.push_back({1, EdgeWeight{1}, EdgeDuration{200}});
    native_result.competitors.push_back({2, EdgeWeight{1}, EdgeDuration{200}});

    const auto result = osrm::engine::isochrone::makeNativeSearchResult(
        facade, native_result, {}, EdgeDuration{100}, 10, false);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1);
    BOOST_REQUIRE_EQUAL(result.competitors.size(), 1);
    BOOST_CHECK_EQUAL(result.nodes.front().node, 0);
    BOOST_CHECK_EQUAL(result.competitors.front().node, 1);
}

BOOST_AUTO_TEST_CASE(inbound_labels_intersect_when_the_geometry_suffix_is_within_cutoff)
{
    const NativeResultFacade facade;
    osrm::engine::routing_algorithms::ReachabilitySearchResult native_result;
    native_result.competitors.push_back({0, EdgeWeight{10}, EdgeDuration{150}});

    const auto result = osrm::engine::isochrone::makeNativeSearchResult(
        facade, native_result, {}, EdgeDuration{60}, 10, true);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1);
    BOOST_CHECK_EQUAL(result.nodes.front().node, 0);
    BOOST_CHECK(result.competitors.empty());
}

BOOST_AUTO_TEST_CASE(enforces_the_retained_search_record_limit)
{
    const NativeResultFacade facade;
    osrm::engine::routing_algorithms::ReachabilitySearchResult native_result;
    native_result.nodes.push_back({0, EdgeWeight{10}, EdgeDuration{50}});
    native_result.nodes.push_back({1, EdgeWeight{10}, EdgeDuration{50}});

    const auto result = osrm::engine::isochrone::makeNativeSearchResult(
        facade, native_result, {}, EdgeDuration{100}, 1, false);

    BOOST_CHECK(result.status ==
                osrm::engine::isochrone::SearchStatus::SearchRecordLimitReached);
}

BOOST_AUTO_TEST_CASE(skips_native_labels_without_materializable_geometry)
{
    const NativeResultFacade facade;
    osrm::engine::routing_algorithms::ReachabilitySearchResult native_result;
    native_result.nodes.push_back({99, EdgeWeight{1}, EdgeDuration{1}});
    native_result.nodes.push_back({0, EdgeWeight{10}, EdgeDuration{50}});

    const auto result = osrm::engine::isochrone::makeNativeSearchResult(
        facade, native_result, {}, EdgeDuration{100}, 10, false);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1);
    BOOST_CHECK_EQUAL(result.nodes.front().node, 0);
}

BOOST_AUTO_TEST_CASE(inbound_skips_native_labels_without_materializable_geometry)
{
    const NativeResultFacade facade;
    osrm::engine::routing_algorithms::ReachabilitySearchResult native_result;
    native_result.competitors.push_back({99, EdgeWeight{1}, EdgeDuration{1}});
    native_result.competitors.push_back({0, EdgeWeight{10}, EdgeDuration{150}});

    const auto result = osrm::engine::isochrone::makeNativeSearchResult(
        facade, native_result, {}, EdgeDuration{60}, 10, true);

    BOOST_REQUIRE(result.isComplete());
    BOOST_REQUIRE_EQUAL(result.nodes.size(), 1);
    BOOST_CHECK_EQUAL(result.nodes.front().node, 0);
}

BOOST_AUTO_TEST_SUITE_END()
