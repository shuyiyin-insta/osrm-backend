#include "server/api/parameters_parser.hpp"

#include "engine/api/isochrone_parameters.hpp"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(api_isochrone_parameters)

using osrm::engine::api::IsochroneParameters;
using osrm::server::api::parseParameters;

BOOST_AUTO_TEST_CASE(requires_exactly_one_coordinate)
{
    auto single_coordinate = parseParameters<IsochroneParameters>("1,2?contours_seconds=300");
    BOOST_REQUIRE(single_coordinate);
    BOOST_CHECK(single_coordinate->IsValid());

    auto multiple_coordinates =
        parseParameters<IsochroneParameters>("1,2;3,4?contours_seconds=300");
    BOOST_REQUIRE(multiple_coordinates);
    BOOST_CHECK(!multiple_coordinates->IsValid());

    auto malformed_coordinates =
        parseParameters<IsochroneParameters>("1,2;invalid?contours_seconds=300");
    BOOST_CHECK(!malformed_coordinates);
}

BOOST_AUTO_TEST_CASE(requires_positive_contours_seconds)
{
    auto contours = parseParameters<IsochroneParameters>("1,2?contours_seconds=1,300.5");
    BOOST_REQUIRE(contours);
    BOOST_REQUIRE_EQUAL(contours->contours_seconds.size(), 2U);
    BOOST_CHECK(contours->IsValid());

    auto empty_contours = parseParameters<IsochroneParameters>("1,2?contours_seconds=");
    BOOST_CHECK(!empty_contours);

    auto zero_contour = parseParameters<IsochroneParameters>("1,2?contours_seconds=0");
    BOOST_REQUIRE(zero_contour);
    BOOST_CHECK(!zero_contour->IsValid());
}

BOOST_AUTO_TEST_CASE(parses_geometry_controls_and_output_formats)
{
    auto json = parseParameters<IsochroneParameters>(
        "1,2.json?contours_seconds=300&direction=inbound&polygons=false&generalize=25&denoise=0.5");
    BOOST_REQUIRE(json);
    BOOST_CHECK(json->IsValid());
    BOOST_CHECK(json->direction == IsochroneParameters::Direction::Inbound);
    BOOST_CHECK(!json->polygons);
    BOOST_REQUIRE(json->generalize);
    BOOST_CHECK_EQUAL(*json->generalize, 25.);
    BOOST_REQUIRE(json->denoise);
    BOOST_CHECK_EQUAL(*json->denoise, 0.5);

    auto flatbuffers = parseParameters<IsochroneParameters>("1,2.flatbuffers?contours_seconds=300");
    BOOST_REQUIRE(flatbuffers);
    BOOST_CHECK(flatbuffers->IsValid());

    auto malformed_control =
        parseParameters<IsochroneParameters>("1,2?contours_seconds=300&direction=sideways");
    BOOST_CHECK(!malformed_control);

    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds=300&contours=300"));
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds=300&range=300"));
}

BOOST_AUTO_TEST_SUITE_END()
