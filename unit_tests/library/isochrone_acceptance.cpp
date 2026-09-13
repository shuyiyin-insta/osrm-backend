#include "coordinates.hpp"
#include "fixture.hpp"

#include "osrm/isochrone_parameters.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include "util/json_renderer.hpp"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

#include <cstddef>
#include <string>
#include <variant>

namespace
{

osrm::IsochroneParameters makeParameters()
{
    osrm::IsochroneParameters parameters;
    parameters.coordinates = {get_dummy_location()};
    parameters.contours_seconds = {300., 60.};
    return parameters;
}

const osrm::json::Array &features(const osrm::json::Object &response)
{ return std::get<osrm::json::Array>(response.values.at("features")); }

void checkFeatureCollection(const osrm::json::Object &response,
                            const osrm::IsochroneParameters &parameters)
{
    BOOST_CHECK_EQUAL(std::get<osrm::json::String>(response.values.at("code")).value, "Ok");
    BOOST_CHECK_EQUAL(std::get<osrm::json::String>(response.values.at("type")).value,
                      "FeatureCollection");

    const auto &response_features = features(response);
    BOOST_REQUIRE_EQUAL(response_features.values.size(), parameters.contours_seconds.size());
    for (std::size_t index = 0; index < response_features.values.size(); ++index)
    {
        const auto &feature = std::get<osrm::json::Object>(response_features.values[index]);
        BOOST_CHECK_EQUAL(std::get<osrm::json::String>(feature.values.at("type")).value, "Feature");

        const auto &properties = std::get<osrm::json::Object>(feature.values.at("properties"));
        BOOST_CHECK_EQUAL(
            std::get<osrm::json::Number>(properties.values.at("contour_seconds")).value,
            parameters.contours_seconds[index]);
        const auto &geometry = std::get<osrm::json::Object>(feature.values.at("geometry"));
        BOOST_CHECK_EQUAL(std::get<osrm::json::String>(geometry.values.at("type")).value,
                          "MultiPolygon");
    }
}

std::string render(const osrm::json::Object &result)
{
    std::string json;
    osrm::util::json::render(json, result);
    return json;
}

} // namespace

BOOST_AUTO_TEST_SUITE(isochrone_acceptance)

BOOST_AUTO_TEST_CASE(ch_and_mld_return_identical_ordered_geojson_contours)
{
    const auto parameters = makeParameters();
    auto ch = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");
    auto mld = getOSRM(OSRM_TEST_DATA_DIR "/mld/monaco.osrm", osrm::EngineConfig::Algorithm::MLD);
    osrm::json::Object ch_result;
    osrm::json::Object mld_result;

    BOOST_REQUIRE(ch.Isochrone(parameters, ch_result) == osrm::Status::Ok);
    BOOST_REQUIRE(mld.Isochrone(parameters, mld_result) == osrm::Status::Ok);
    checkFeatureCollection(ch_result, parameters);
    checkFeatureCollection(mld_result, parameters);

    // This guards both algorithmic reachability parity and deterministic contour materialization.
    BOOST_CHECK_EQUAL(render(ch_result), render(mld_result));
}

BOOST_AUTO_TEST_CASE(rejects_a_contour_above_the_configured_maximum_without_clamping)
{
    osrm::EngineConfig config;
    config.storage_config = {OSRM_TEST_DATA_DIR "/mld/monaco.osrm"};
    config.use_shared_memory = false;
    config.algorithm = osrm::EngineConfig::Algorithm::MLD;
    config.max_isochrone_range = 60;
    const osrm::OSRM routing_machine{config};

    auto parameters = makeParameters();
    parameters.contours_seconds = {60.1};
    osrm::json::Object result;

    BOOST_CHECK(routing_machine.Isochrone(parameters, result) == osrm::Status::Error);
    BOOST_CHECK_EQUAL(std::get<osrm::json::String>(result.values.at("code")).value,
                      "InvalidOptions");
}

BOOST_AUTO_TEST_CASE(accepts_the_configured_maximum_without_altering_it)
{
    osrm::EngineConfig config;
    config.storage_config = {OSRM_TEST_DATA_DIR "/mld/monaco.osrm"};
    config.use_shared_memory = false;
    config.algorithm = osrm::EngineConfig::Algorithm::MLD;
    config.max_isochrone_range = 60;
    const osrm::OSRM routing_machine{config};

    auto parameters = makeParameters();
    parameters.contours_seconds = {60.};
    osrm::json::Object result;

    BOOST_REQUIRE(routing_machine.Isochrone(parameters, result) == osrm::Status::Ok);
    checkFeatureCollection(result, parameters);
}

BOOST_AUTO_TEST_SUITE_END()
