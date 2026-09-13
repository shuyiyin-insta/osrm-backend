#include "engine/plugins/isochrone.hpp"

#include "engine/api/isochrone_api.hpp"
#include "engine/api/isochrone_parameters.hpp"
#include "engine/isochrone/duration.hpp"
#include "engine/isochrone/native_search_result.hpp"
#include "engine/isochrone/search_result_materialization.hpp"
#include "engine/isochrone/weighted_polyline_grid.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <variant>

namespace osrm::engine::plugins
{

Status IsochronePlugin::HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                                      const api::IsochroneParameters &parameters,
                                      osrm::engine::api::ResultT &result) const
{
    if (!parameters.IsValid())
        return Error("InvalidOptions", "Invalid isochrone parameters.", result);

    if (!CheckAllCoordinates(parameters.coordinates))
        return Error("InvalidOptions", "Coordinates are invalid", result);

    if (!CheckAlgorithms(parameters, algorithms, result))
        return Status::Error;

    if (parameters.format != api::BaseParameters::OutputFormatType::JSON ||
        !std::holds_alternative<util::json::Object>(result))
    {
        return Error(
            "NotImplemented", "The isochrone service only supports JSON/GeoJSON output.", result);
    }

    if (parameters.contours_seconds.size() > MAX_CONTOURS)
    {
        return Error("TooBig",
                     "Number of contours_seconds is higher than current maximum (" +
                         std::to_string(MAX_CONTOURS) + ")",
                     result);
    }

    EdgeDuration maximum_duration{0};
    for (const auto contour_seconds : parameters.contours_seconds)
    {
        if (contour_seconds > static_cast<double>(max_range_seconds))
        {
            return Error("InvalidOptions",
                         "Contour exceeds the configured maximum isochrone range.",
                         result);
        }

        const auto contour_duration = isochrone::durationCutoffFromSeconds(contour_seconds);
        if (!contour_duration || *contour_duration < EdgeDuration{1})
        {
            const auto truncated_contour =
                std::floor(contour_seconds * isochrone::INTERNAL_DURATION_UNITS_PER_SECOND);
            if (std::isfinite(truncated_contour) && truncated_contour < 1.)
            {
                return Error("InvalidValue",
                             "Contour is below the supported duration precision of 0.1 seconds.",
                             result);
            }
            return Error("InvalidValue",
                         "Contour exceeds the maximum supported duration in seconds.",
                         result);
        }
        maximum_duration = std::max(maximum_duration, *contour_duration);
    }

    const auto &facade = algorithms.GetFacade();
    const auto inbound = parameters.direction == api::IsochroneParameters::Direction::Inbound;
    const auto role = inbound ? area::ApproachRole::Arrival : area::ApproachRole::Departure;
    auto phantom_node_pairs = GetPhantomNodesForRole(facade, parameters, role);
    if (phantom_node_pairs.size() != parameters.coordinates.size())
    {
        return Error("NoSegment",
                     MissingPhantomErrorMessage(phantom_node_pairs, parameters.coordinates),
                     result);
    }

    auto snapped_phantoms = SnapPhantomNodes(std::move(phantom_node_pairs));
    BOOST_ASSERT(snapped_phantoms.size() == 1);
    const auto &endpoint_candidates = snapped_phantoms.front();

    const auto native_result = algorithms.IsochroneSearch(
        endpoint_candidates, maximum_duration, inbound, MAX_SEARCH_NODES);
    switch (native_result.status)
    {
    case routing_algorithms::ReachabilitySearchStatus::Complete:
        break;
    case routing_algorithms::ReachabilitySearchStatus::SearchNodeLimitReached:
        return Error("TooBig", "Isochrone search exceeds the configured node limit.", result);
    case routing_algorithms::ReachabilitySearchStatus::UnsupportedGraph:
        return Error(
            "NotImplemented", "The loaded graph does not support isochrone search.", result);
    case routing_algorithms::ReachabilitySearchStatus::ArithmeticOverflow:
        return Error("InternalError", "Isochrone search encountered arithmetic overflow.", result);
    }

    const auto search_result = isochrone::makeNativeSearchResult(
        facade, native_result, endpoint_candidates, maximum_duration, MAX_SEARCH_RECORDS, inbound);
    switch (search_result.status)
    {
    case isochrone::SearchStatus::Complete:
        break;
    case isochrone::SearchStatus::SearchRecordLimitReached:
        return Error(
            "TooBig", "Isochrone search exceeded the maximum number of search records.", result);
    case isochrone::SearchStatus::ArithmeticOverflow:
        return Error("InternalError", "Unable to prepare isochrone geometry.", result);
    }

    const isochrone::MaterializationLimits materialization_limits{MAX_MATERIALIZED_POINTS,
                                                                  MAX_MATERIALIZED_POINTS,
                                                                  MAX_MATERIALIZED_POINTS,
                                                                  MAX_MATERIALIZED_POINTS};
    const auto materialization_result = isochrone::materializeSearchResult(
        facade, search_result, maximum_duration, inbound, materialization_limits);
    switch (materialization_result.error)
    {
    case isochrone::MaterializationError::None:
        break;
    case isochrone::MaterializationError::BudgetExceeded:
        return Error("TooBig", "Isochrone geometry exceeds the configured limit.", result);
    case isochrone::MaterializationError::RequiresLongitudeWrap:
        return Error("NotImplemented",
                     "Isochrone geometry crossing the antimeridian is not supported.",
                     result);
    case isochrone::MaterializationError::TouchesPole:
        return Error("NotImplemented",
                     "Isochrone geometry touching a geographic pole is not supported.",
                     result);
    default:
        return Error("InternalError", "Unable to materialize isochrone geometry.", result);
    }

    const auto rasterization =
        isochrone::rasterizeWeightedPolylines(materialization_result.polylines,
                                              {100., MAX_GRID_CELLS, MAX_RASTERIZATION_STEPS},
                                              parameters.coordinates.front());
    switch (rasterization.error)
    {
    case isochrone::RasterizationError::None:
        break;
    case isochrone::RasterizationError::InvalidOptions:
        return Error("InternalError", "Isochrone rasterization configuration is invalid.", result);
    case isochrone::RasterizationError::TooBig:
        return Error("TooBig", "Isochrone rasterization exceeds the configured limit.", result);
    case isochrone::RasterizationError::TouchesLongitudeBoundary:
        return Error("NotImplemented",
                     "Isochrone geometry at the longitude world boundary is not supported.",
                     result);
    case isochrone::RasterizationError::TouchesPole:
        return Error("NotImplemented",
                     "Isochrone geometry touching a geographic pole is not supported.",
                     result);
    }
    BOOST_ASSERT(rasterization.grid);

    api::IsochroneAPI isochrone_api{facade, parameters};
    auto &json_result = std::get<util::json::Object>(result);
    if (isochrone_api.MakeResponse(
            *rasterization.grid, endpoint_candidates, MAX_OUTPUT_POINTS, json_result) ==
        api::IsochroneAPI::ResponseStatus::TooBig)
    {
        return Error("TooBig", "Isochrone response exceeds the configured limit.", result);
    }

    return Status::Ok;
}

} // namespace osrm::engine::plugins
