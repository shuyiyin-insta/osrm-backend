#include "coordinates.hpp"
#include "fixture.hpp"

#include "osrm/isochrone_parameters.hpp"
#include "osrm/json_container.hpp"
#include "osrm/osrm.hpp"
#include "osrm/status.hpp"

#include <boost/test/test_tools.hpp>
#include <boost/test/unit_test.hpp>

namespace
{

void checkInvalidParameters(const osrm::OSRM &routing_machine,
                            const osrm::IsochroneParameters &parameters)
{
    osrm::engine::api::ResultT result = osrm::json::Object();

    const auto status = routing_machine.Isochrone(parameters, result);

    BOOST_CHECK(status == osrm::Status::Error);
    const auto &json_result = std::get<osrm::json::Object>(result);
    BOOST_CHECK_EQUAL(std::get<osrm::json::String>(json_result.values.at("code")).value,
                      "InvalidOptions");
}

} // namespace

BOOST_AUTO_TEST_SUITE(isochrone)

BOOST_AUTO_TEST_CASE(rejects_invalid_parameters_before_accessing_the_start_coordinate)
{
    auto routing_machine = getOSRM(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");

    osrm::IsochroneParameters no_coordinates;
    checkInvalidParameters(routing_machine, no_coordinates);

    osrm::IsochroneParameters multiple_coordinates;
    multiple_coordinates.coordinates = {get_dummy_location(), get_dummy_location()};
    checkInvalidParameters(routing_machine, multiple_coordinates);

    osrm::IsochroneParameters no_contours;
    no_contours.coordinates = {get_dummy_location()};
    checkInvalidParameters(routing_machine, no_contours);

    osrm::IsochroneParameters zero_contour;
    zero_contour.coordinates = {get_dummy_location()};
    zero_contour.contours_seconds = {0.};
    checkInvalidParameters(routing_machine, zero_contour);
}

BOOST_AUTO_TEST_SUITE_END()
