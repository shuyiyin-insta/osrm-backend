#include "server/api/parameters_parser.hpp"

#include "engine/api/isochrone_parameters.hpp"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(api_isochrone_parameters)

using osrm::engine::api::IsochroneParameters;
using osrm::server::api::parseParameters;

BOOST_AUTO_TEST_CASE(requires_exactly_one_coordinate)
{
    auto single_coordinate = parseParameters<IsochroneParameters>("1,2");
    BOOST_REQUIRE(single_coordinate);
    BOOST_CHECK(single_coordinate->IsValid());

    auto multiple_coordinates = parseParameters<IsochroneParameters>("1,2;3,4");
    BOOST_REQUIRE(multiple_coordinates);
    BOOST_CHECK(!multiple_coordinates->IsValid());

    auto malformed_coordinates = parseParameters<IsochroneParameters>("1,2;invalid");
    BOOST_CHECK(!malformed_coordinates);
}

BOOST_AUTO_TEST_CASE(requires_a_positive_range)
{
    auto minimum_range = parseParameters<IsochroneParameters>("1,2?range=1");
    BOOST_REQUIRE(minimum_range);
    BOOST_CHECK(minimum_range->IsValid());

    auto zero_range = parseParameters<IsochroneParameters>("1,2?range=0");
    BOOST_REQUIRE(zero_range);
    BOOST_CHECK(!zero_range->IsValid());
}

BOOST_AUTO_TEST_CASE(rejects_unsupported_output_formats)
{
    auto json = parseParameters<IsochroneParameters>("1,2.json");
    BOOST_REQUIRE(json);
    BOOST_CHECK(json->IsValid());

    auto flatbuffers = parseParameters<IsochroneParameters>("1,2.flatbuffers");
    BOOST_CHECK(!flatbuffers);
}

BOOST_AUTO_TEST_SUITE_END()
