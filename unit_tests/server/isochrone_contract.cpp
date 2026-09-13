#include "server/api/parameters_parser.hpp"
#include "server/api/url_parser.hpp"

#include "osrm/isochrone_parameters.hpp"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

#include <cmath>
#include <limits>

namespace
{

using osrm::IsochroneParameters;
using osrm::server::api::parseParameters;

} // namespace

BOOST_AUTO_TEST_SUITE(isochrone_contract)

BOOST_AUTO_TEST_CASE(parses_explicit_duration_contours_and_preserves_request_order)
{
    const auto parameters = parseParameters<IsochroneParameters>(
        "1,2?contours_seconds=600,300.5&direction=inbound&polygons=false&generalize=12.5&"
        "denoise=0.25&radiuses=60&bearings=200,10&approaches=curb&exclude=ferry,motorway");

    BOOST_REQUIRE(parameters);
    BOOST_REQUIRE(parameters->IsValid());
    BOOST_REQUIRE_EQUAL(parameters->coordinates.size(), 1U);
    BOOST_REQUIRE_EQUAL(parameters->contours_seconds.size(), 2U);
    BOOST_CHECK_EQUAL(parameters->contours_seconds[0], 600.);
    BOOST_CHECK_EQUAL(parameters->contours_seconds[1], 300.5);
    BOOST_CHECK(parameters->direction == IsochroneParameters::Direction::Inbound);
    BOOST_CHECK(!parameters->polygons);
    BOOST_REQUIRE(parameters->generalize);
    BOOST_CHECK_EQUAL(*parameters->generalize, 12.5);
    BOOST_REQUIRE(parameters->denoise);
    BOOST_CHECK_EQUAL(*parameters->denoise, 0.25);
}

BOOST_AUTO_TEST_CASE(defaults_are_outbound_polygon_contours)
{
    const auto parameters = parseParameters<IsochroneParameters>("1,2?contours_seconds=300");

    BOOST_REQUIRE(parameters);
    BOOST_REQUIRE(parameters->IsValid());
    BOOST_CHECK(parameters->direction == IsochroneParameters::Direction::Outbound);
    BOOST_CHECK(parameters->polygons);
    BOOST_CHECK(!parameters->generalize);
    BOOST_CHECK(!parameters->denoise);
}

BOOST_AUTO_TEST_CASE(requires_one_or_more_positive_finite_duration_contours)
{
    const auto missing = parseParameters<IsochroneParameters>("1,2");
    BOOST_REQUIRE(missing);
    BOOST_CHECK(!missing->IsValid());

    const auto multiple_coordinates =
        parseParameters<IsochroneParameters>("1,2;3,4?contours_seconds=300");
    BOOST_REQUIRE(multiple_coordinates);
    BOOST_CHECK(!multiple_coordinates->IsValid());

    const auto non_positive = parseParameters<IsochroneParameters>("1,2?contours_seconds=0,-1");
    BOOST_REQUIRE(non_positive);
    BOOST_CHECK(!non_positive->IsValid());

    IsochroneParameters non_finite;
    non_finite.coordinates = {{{osrm::util::FloatLongitude{1}, osrm::util::FloatLatitude{2}}}};
    non_finite.contours_seconds = {std::numeric_limits<double>::infinity()};
    BOOST_CHECK(!non_finite.IsValid());
}

BOOST_AUTO_TEST_CASE(rejects_unknown_or_malformed_isochrone_options)
{
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours=300"));
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds="));
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds=300,nope"));
    BOOST_CHECK(
        !parseParameters<IsochroneParameters>("1,2?contours_seconds=300&direction=sideways"));
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds=300&polygons=maybe"));
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds=300&generalize=nan"));
    BOOST_CHECK(!parseParameters<IsochroneParameters>("1,2?contours_seconds=300&denoise=inf"));

    const auto negative_generalize =
        parseParameters<IsochroneParameters>("1,2?contours_seconds=300&generalize=-1");
    BOOST_REQUIRE(negative_generalize);
    BOOST_CHECK(!negative_generalize->IsValid());

    const auto excessive_denoise =
        parseParameters<IsochroneParameters>("1,2?contours_seconds=300&denoise=1.01");
    BOOST_REQUIRE(excessive_denoise);
    BOOST_CHECK(!excessive_denoise->IsValid());
}

BOOST_AUTO_TEST_CASE(uses_the_standard_versioned_isochrone_url)
{
    const auto url = osrm::server::api::parseURL(
        "/isochrone/v1/driving/1,2?contours_seconds=600,300&direction=inbound");

    BOOST_REQUIRE(url);
    BOOST_CHECK_EQUAL(url->service, "isochrone");
    BOOST_CHECK_EQUAL(url->version, 1U);
    BOOST_CHECK_EQUAL(url->profile, "driving");
}

BOOST_AUTO_TEST_SUITE_END()
