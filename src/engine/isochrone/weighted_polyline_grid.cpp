#include "engine/isochrone/weighted_polyline_grid.hpp"

#include "engine/isochrone/cell_contours.hpp"
#include "engine/isochrone/grid_polygon_denoising.hpp"
#include "engine/isochrone/grid_polygon_generalization.hpp"

#include <boost/assert.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

namespace osrm::engine::isochrone
{
namespace
{

struct Bounds
{
    double min_lon = std::numeric_limits<double>::infinity();
    double max_lon = -std::numeric_limits<double>::infinity();
    double min_lat = std::numeric_limits<double>::infinity();
    double max_lat = -std::numeric_limits<double>::infinity();
};

struct MetricPoint
{
    double x;
    double y;
};

struct MetricBounds
{
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
};

struct GridBounds
{
    std::int64_t min_x;
    std::int64_t max_x;
    std::int64_t min_y;
    std::int64_t max_y;
};

bool isUsable(const WeightedPolylinePoint &point)
{
    return point.coordinate.IsValid() && std::isfinite(point.duration) &&
           std::isfinite(point.weight);
}

double longitude(const util::Coordinate coordinate)
{ return static_cast<double>(util::toFloating(coordinate.lon)); }

double latitude(const util::Coordinate coordinate)
{ return static_cast<double>(util::toFloating(coordinate.lat)); }

// This spherical equirectangular projection is a locally metric, reversible
// mapping.  Isochrone requests use their source coordinate as its reference,
// so adding farther geometry cannot move existing contour cell boundaries.
constexpr double EARTH_MEAN_RADIUS_METRES = 6'371'008.8;
constexpr double PI = 3.14159265358979323846;
constexpr double METRES_PER_DEGREE_LATITUDE = EARTH_MEAN_RADIUS_METRES * PI / 180.;
constexpr double MINIMUM_CELL_SIZE_METRES = METRES_PER_DEGREE_LATITUDE / COORDINATE_PRECISION;

struct LocalProjection
{
    double reference_lon;
    double reference_lat;
    double metres_per_degree_lon;

    MetricPoint project(const util::Coordinate coordinate) const
    {
        return {(longitude(coordinate) - reference_lon) * metres_per_degree_lon,
                (latitude(coordinate) - reference_lat) * METRES_PER_DEGREE_LATITUDE};
    }

    double longitudeAt(const double x) const { return reference_lon + x / metres_per_degree_lon; }

    double latitudeAt(const double y) const
    { return reference_lat + y / METRES_PER_DEGREE_LATITUDE; }
};

LocalProjection makeProjection(const util::Coordinate reference)
{
    const auto reference_lon = longitude(reference);
    const auto reference_lat = latitude(reference);
    return {reference_lon,
            reference_lat,
            METRES_PER_DEGREE_LATITUDE * std::cos(reference_lat * PI / 180.)};
}

LocalProjection makeProjection(const WeightedGrid &grid)
{
    const auto reference_lon = static_cast<double>(grid.reference_lon);
    const auto reference_lat = static_cast<double>(grid.reference_lat);
    return {reference_lon,
            reference_lat,
            METRES_PER_DEGREE_LATITUDE * std::cos(reference_lat * PI / 180.)};
}

std::optional<GridBounds> gridBounds(const MetricBounds &bounds, const double cell_size)
{
    const auto cell_size_as_long_double = static_cast<long double>(cell_size);
    const auto toCellIndex =
        [cell_size_as_long_double](const double coordinate) -> std::optional<std::int64_t>
    {
        const auto index =
            std::floor(static_cast<long double>(coordinate) / cell_size_as_long_double);
        constexpr auto minimum = static_cast<long double>(std::numeric_limits<std::int64_t>::min());
        constexpr auto past_maximum = -minimum;
        if (!std::isfinite(index) || index < minimum || index >= past_maximum)
            return std::nullopt;
        return static_cast<std::int64_t>(index);
    };

    const auto min_x = toCellIndex(bounds.min_x);
    const auto max_x = toCellIndex(bounds.max_x);
    const auto min_y = toCellIndex(bounds.min_y);
    const auto max_y = toCellIndex(bounds.max_y);
    if (!min_x || !max_x || !min_y || !max_y)
        return std::nullopt;
    return GridBounds{*min_x, *max_x, *min_y, *max_y};
}

std::optional<std::size_t> dimension(const std::int64_t minimum, const std::int64_t maximum)
{
    if (maximum < minimum)
        return std::nullopt;

    const auto distance = static_cast<std::uint64_t>(maximum) - static_cast<std::uint64_t>(minimum);
    if (distance >= std::numeric_limits<std::size_t>::max())
        return std::nullopt;
    return static_cast<std::size_t>(distance + 1);
}

bool exceedsCellCap(const GridBounds &bounds, const std::size_t maximum_cell_count)
{
    const auto width = dimension(bounds.min_x, bounds.max_x);
    const auto height = dimension(bounds.min_y, bounds.max_y);
    return !width || !height || *width > maximum_cell_count / *height;
}

RasterizationError rasterizationExtentError(const GridBounds &bounds,
                                            const double cell_size,
                                            const LocalProjection &projection)
{
    // Allow only binary floating-point round-off at longitude endpoints.  A
    // metre cell that extends beyond the geographic world would otherwise
    // produce an invalid inverse projection; contour construction cannot
    // report an error, so never clamp those vertices.
    constexpr long double endpoint_epsilon = 1e-12L;
    const auto minimum_x = static_cast<double>(static_cast<long double>(bounds.min_x) * cell_size);
    const auto maximum_x =
        static_cast<double>((static_cast<long double>(bounds.max_x) + 1.) * cell_size);
    const auto minimum_y = static_cast<double>(static_cast<long double>(bounds.min_y) * cell_size);
    const auto maximum_y =
        static_cast<double>((static_cast<long double>(bounds.max_y) + 1.) * cell_size);
    const auto minimum_latitude = projection.latitudeAt(minimum_y);
    const auto maximum_latitude = projection.latitudeAt(maximum_y);
    if (minimum_latitude <= -90. || maximum_latitude >= 90.)
        return RasterizationError::TouchesPole;

    const auto minimum_longitude = projection.longitudeAt(minimum_x);
    const auto maximum_longitude = projection.longitudeAt(maximum_x);
    if (minimum_longitude < -180. - endpoint_epsilon || maximum_longitude > 180. + endpoint_epsilon)
        return RasterizationError::TouchesLongitudeBoundary;

    return RasterizationError::None;
}

double gridPosition(const double value,
                    const double origin,
                    const double cell_size,
                    const std::size_t dimension)
{
    const auto position = (value - origin) / cell_size;
    return std::clamp(position, 0., std::nextafter(static_cast<double>(dimension), 0.));
}

struct PolylineSegment
{
    const WeightedPolylinePoint *from;
    const WeightedPolylinePoint *to;

    double begin() const { return from->geometry_position.value_or(0.); }
    double end() const { return to->geometry_position.value_or(1.); }
};

struct PositionInterval
{
    double begin;
    double end;
    bool include_begin = true;
    bool include_end = true;
};

struct SelectedSegment
{
    std::size_t segment;
    PositionInterval interval;
};

long double fractionAt(const PolylineSegment &segment, const double position)
{
    if (segment.begin() == segment.end())
        return 0.;
    return std::clamp((static_cast<long double>(position) - segment.begin()) /
                          (static_cast<long double>(segment.end()) - segment.begin()),
                      0.L,
                      1.L);
}

std::pair<long double, long double> labelAt(const PolylineSegment &segment, const double position)
{
    const auto fraction = fractionAt(segment, position);
    if (fraction == 0.)
        return {segment.from->weight, segment.from->duration};
    if (fraction == 1.)
        return {segment.to->weight, segment.to->duration};
    return {segment.from->weight +
                (static_cast<long double>(segment.to->weight) - segment.from->weight) * fraction,
            segment.from->duration +
                (static_cast<long double>(segment.to->duration) - segment.from->duration) *
                    fraction};
}

WeightedPolylinePoint pointAt(const PolylineSegment &segment, const double position)
{
    const auto fraction = fractionAt(segment, position);
    if (fraction == 0.)
        return *segment.from;
    if (fraction == 1.)
        return *segment.to;

    const auto [weight, duration] = labelAt(segment, position);
    return {{util::FloatLongitude{
                 static_cast<double>(longitude(segment.from->coordinate) +
                                     (static_cast<long double>(longitude(segment.to->coordinate)) -
                                      longitude(segment.from->coordinate)) *
                                         fraction)},
             util::FloatLatitude{
                 static_cast<double>(latitude(segment.from->coordinate) +
                                     (static_cast<long double>(latitude(segment.to->coordinate)) -
                                      latitude(segment.from->coordinate)) *
                                         fraction)}},
            static_cast<double>(duration),
            static_cast<double>(weight),
            segment.from->geometry_id,
            segment.from->geometry_position ? std::optional<double>{position} : std::nullopt};
}

std::optional<PositionInterval> losingInterval(const PolylineSegment &candidate,
                                               const PolylineSegment &competitor,
                                               const PositionInterval overlap,
                                               const bool competitor_first)
{
    const auto candidate_begin = labelAt(candidate, overlap.begin);
    const auto competitor_begin = labelAt(competitor, overlap.begin);
    if (overlap.begin == overlap.end)
    {
        return competitor_begin < candidate_begin ||
                       (competitor_begin == candidate_begin && competitor_first)
                   ? std::optional<PositionInterval>{overlap}
                   : std::nullopt;
    }

    const auto candidate_end = labelAt(candidate, overlap.end);
    const auto competitor_end = labelAt(competitor, overlap.end);
    const auto make_interval = [&](const double begin, const double end)
    {
        const auto competitor_wins_at = [&](const double position)
        {
            const auto candidate_label = labelAt(candidate, position);
            const auto competitor_label = labelAt(competitor, position);
            return competitor_label < candidate_label ||
                   (competitor_label == candidate_label && competitor_first);
        };
        return PositionInterval{begin, end, competitor_wins_at(begin), competitor_wins_at(end)};
    };
    auto begin_difference = candidate_begin.first - competitor_begin.first;
    auto end_difference = candidate_end.first - competitor_end.first;
    if (begin_difference == 0. && end_difference == 0.)
    {
        begin_difference = candidate_begin.second - competitor_begin.second;
        end_difference = candidate_end.second - competitor_end.second;
        if (begin_difference == 0. && end_difference == 0.)
            return competitor_first ? std::optional<PositionInterval>{overlap} : std::nullopt;
    }
    if (begin_difference <= 0. && end_difference <= 0.)
    {
        if (begin_difference == 0. && (competitor_begin < candidate_begin ||
                                       (competitor_begin == candidate_begin && competitor_first)))
            return PositionInterval{overlap.begin, overlap.begin};
        if (end_difference == 0. && (competitor_end < candidate_end ||
                                     (competitor_end == candidate_end && competitor_first)))
            return PositionInterval{overlap.end, overlap.end};
        return std::nullopt;
    }
    if (begin_difference >= 0. && end_difference >= 0.)
        return make_interval(overlap.begin, overlap.end);

    const auto crossing = static_cast<double>(
        overlap.begin + (static_cast<long double>(overlap.end) - overlap.begin) * begin_difference /
                            (begin_difference - end_difference));
    return begin_difference > 0. ? make_interval(overlap.begin, crossing)
                                 : make_interval(crossing, overlap.end);
}

RasterizationError prepareSegments(const std::span<const WeightedPolyline> polylines,
                                   const WeightedGridOptions &options,
                                   std::vector<PolylineSegment> &segments,
                                   std::vector<SelectedSegment> &selected)
{
    for (const auto &polyline : polylines)
    {
        std::optional<std::optional<PackedGeometryID>> geometry_id;
        const WeightedPolylinePoint *previous = nullptr;
        for (const auto &point : polyline)
        {
            if (!isUsable(point))
            {
                previous = nullptr;
                continue;
            }
            if (!geometry_id)
                geometry_id = point.geometry_id;
            else if (*geometry_id != point.geometry_id)
                return RasterizationError::InvalidOptions;
            if (point.geometry_id &&
                (!point.geometry_position || !std::isfinite(*point.geometry_position) ||
                 *point.geometry_position < 0.))
                return RasterizationError::InvalidOptions;
            if (segments.size() >= options.maximum_rasterization_steps)
                return RasterizationError::TooBig;

            auto from = previous ? previous : &point;
            auto to = &point;
            if (point.geometry_id && *from->geometry_position > *to->geometry_position)
                std::swap(from, to);
            segments.push_back({from, to});
            previous = &point;
        }
    }

    const auto append = [&](const std::size_t segment_index, PositionInterval interval)
    {
        const auto &segment = segments[segment_index];
        const auto begin_duration = labelAt(segment, interval.begin).second;
        const auto end_duration = labelAt(segment, interval.end).second;
        if (begin_duration > options.maximum_duration && end_duration > options.maximum_duration)
            return true;
        if (begin_duration > options.maximum_duration || end_duration > options.maximum_duration)
        {
            const auto crossing = static_cast<double>(
                interval.begin + (static_cast<long double>(interval.end) - interval.begin) *
                                     (options.maximum_duration - begin_duration) /
                                     (end_duration - begin_duration));
            if (begin_duration > options.maximum_duration)
            {
                interval.include_begin = crossing == interval.end     ? interval.include_end
                                         : crossing == interval.begin ? interval.include_begin
                                                                      : true;
                interval.begin = crossing;
            }
            else
            {
                interval.include_end = crossing == interval.begin ? interval.include_begin
                                       : crossing == interval.end ? interval.include_end
                                                                  : true;
                interval.end = crossing;
            }
        }
        if (interval.begin == interval.end && (!interval.include_begin || !interval.include_end))
            return true;
        if (selected.size() >= options.maximum_rasterization_steps)
            return false;
        selected.push_back({segment_index, interval});
        return true;
    };

    std::vector<std::size_t> order(segments.size());
    std::iota(order.begin(), order.end(), 0);
    std::sort(order.begin(),
              order.end(),
              [&](const auto left, const auto right)
              {
                  const auto &a = segments[left];
                  const auto &b = segments[right];
                  return std::tuple{a.from->geometry_id, a.begin(), a.end(), left} <
                         std::tuple{b.from->geometry_id, b.begin(), b.end(), right};
              });

    std::size_t comparisons = 0;
    std::vector<double> maximum_ends;
    std::vector<PositionInterval> intervals;
    std::vector<PositionInterval> remaining;
    for (std::size_t group_begin = 0; group_begin < order.size();)
    {
        const auto geometry_id = segments[order[group_begin]].from->geometry_id;
        if (!geometry_id)
        {
            const auto index = order[group_begin++];
            if (!append(index, {segments[index].begin(), segments[index].end()}))
                return RasterizationError::TooBig;
            continue;
        }
        auto group_end = group_begin + 1;
        while (group_end < order.size() &&
               segments[order[group_end]].from->geometry_id == geometry_id)
            ++group_end;

        maximum_ends.clear();
        auto maximum_end = 0.;
        for (auto index = group_begin; index < group_end; ++index)
        {
            maximum_end = std::max(maximum_end, segments[order[index]].end());
            maximum_ends.push_back(maximum_end);
        }
        for (auto index = group_begin; index < group_end; ++index)
        {
            const auto candidate_index = order[index];
            const auto &candidate = segments[candidate_index];
            intervals.assign(1, {candidate.begin(), candidate.end()});
            const auto first =
                std::lower_bound(maximum_ends.begin(), maximum_ends.end(), candidate.begin());
            const auto first_index =
                group_begin + static_cast<std::size_t>(first - maximum_ends.begin());
            for (auto other = first_index; other < group_end && !intervals.empty(); ++other)
            {
                const auto competitor_index = order[other];
                const auto &competitor = segments[competitor_index];
                if (competitor.begin() > candidate.end())
                    break;
                if (competitor_index == candidate_index)
                    continue;
                if (comparisons >= options.maximum_rasterization_steps)
                    return RasterizationError::TooBig;
                const PositionInterval overlap{std::max(candidate.begin(), competitor.begin()),
                                               std::min(candidate.end(), competitor.end())};
                const auto losing =
                    overlap.begin <= overlap.end
                        ? losingInterval(
                              candidate, competitor, overlap, competitor_index < candidate_index)
                        : std::nullopt;
                remaining.clear();
                for (const auto interval : intervals)
                {
                    if (comparisons >= options.maximum_rasterization_steps)
                        return RasterizationError::TooBig;
                    ++comparisons;
                    if (!losing || interval.end < losing->begin || losing->end < interval.begin)
                        remaining.push_back(interval);
                    else
                    {
                        const auto wins_at = [&](const double position)
                        {
                            return position < losing->begin || losing->end < position ||
                                   (position == losing->begin && !losing->include_begin) ||
                                   (position == losing->end && !losing->include_end);
                        };
                        if (interval.begin < losing->begin)
                            remaining.push_back({interval.begin,
                                                 losing->begin,
                                                 interval.include_begin,
                                                 wins_at(losing->begin)});
                        else if (interval.include_begin && wins_at(interval.begin))
                            remaining.push_back({interval.begin, interval.begin});
                        if (losing->end < interval.end)
                            remaining.push_back({losing->end,
                                                 interval.end,
                                                 wins_at(losing->end),
                                                 interval.include_end});
                        else if (interval.end != interval.begin && interval.include_end &&
                                 wins_at(interval.end))
                            remaining.push_back({interval.end, interval.end});
                    }
                }
                intervals.swap(remaining);
            }
            for (const auto interval : intervals)
            {
                if (!append(candidate_index, interval))
                    return RasterizationError::TooBig;
            }
        }
        group_begin = group_end;
    }
    return RasterizationError::None;
}

double durationAt(const WeightedPolylinePoint &from,
                  const WeightedPolylinePoint &to,
                  const double fraction)
{
    if (fraction == 0.)
        return from.duration;
    if (fraction == 1.)
        return to.duration;
    return from.duration + (to.duration - from.duration) * fraction;
}

template <typename UpdateCell>
bool rasterizeSegment(const WeightedGrid &grid,
                      const WeightedPolylinePoint &from,
                      const WeightedPolylinePoint &to,
                      const bool include_from,
                      const bool include_to,
                      UpdateCell &&update_cell)
{
    const auto projection = makeProjection(grid);
    const auto from_point = projection.project(from.coordinate);
    const auto to_point = projection.project(to.coordinate);
    auto x0 = gridPosition(from_point.x, grid.origin_x, grid.cell_size, grid.width);
    auto y0 = gridPosition(from_point.y, grid.origin_y, grid.cell_size, grid.height);
    auto x1 = gridPosition(to_point.x, grid.origin_x, grid.cell_size, grid.width);
    auto y1 = gridPosition(to_point.y, grid.origin_y, grid.cell_size, grid.height);
    if (!include_from)
    {
        x0 = std::nextafter(x0, x1);
        y0 = std::nextafter(y0, y1);
    }
    if (!include_to)
    {
        x1 = std::nextafter(x1, x0);
        y1 = std::nextafter(y1, y0);
    }

    const auto dx = x1 - x0;
    const auto dy = y1 - y0;
    auto x = static_cast<std::size_t>(std::floor(x0));
    auto y = static_cast<std::size_t>(std::floor(y0));
    const auto end_x = static_cast<std::size_t>(std::floor(x1));
    const auto end_y = static_cast<std::size_t>(std::floor(y1));

    if (dx == 0. && dy == 0.)
    {
        return update_cell(x, y, std::min(from.duration, to.duration));
    }

    const auto step_x = dx > 0. ? 1 : dx < 0. ? -1 : 0;
    const auto step_y = dy > 0. ? 1 : dy < 0. ? -1 : 0;
    const auto infinity = std::numeric_limits<double>::infinity();
    const auto t_delta_x = step_x == 0 ? infinity : std::abs(1. / dx);
    const auto t_delta_y = step_y == 0 ? infinity : std::abs(1. / dy);
    auto t_max_x = infinity;
    auto t_max_y = infinity;
    if (step_x > 0)
        t_max_x = (std::floor(x0) + 1. - x0) / dx;
    else if (step_x < 0)
        t_max_x = (x0 - std::floor(x0)) / -dx;
    if (step_y > 0)
        t_max_y = (std::floor(y0) + 1. - y0) / dy;
    else if (step_y < 0)
        t_max_y = (y0 - std::floor(y0)) / -dy;

    auto t = 0.;
    for (std::size_t step = 0; step <= grid.width + grid.height; ++step)
    {
        const auto next_t = std::min({t_max_x, t_max_y, 1.});
        const auto next_label = durationAt(from, to, next_t);
        if (!update_cell(x, y, std::min(durationAt(from, to, t), next_label)))
        {
            return false;
        }

        if (x == end_x && y == end_y)
            return true;

        const auto crosses_x = t_max_x <= t_max_y && t_max_x <= 1.;
        const auto crosses_y = t_max_y <= t_max_x && t_max_y <= 1.;

        if (next_t == 1.)
        {
            auto endpoint_updated = false;
            if (crosses_x && crosses_y)
            {
                // An endpoint on a grid corner can have a supercover neighbour outside the
                // bounded grid. Mark the in-bounds neighbours, then finish at the endpoint
                // without advancing the DDA beyond the segment.
                const auto side_x = static_cast<std::int64_t>(x) + step_x;
                const auto side_y = static_cast<std::int64_t>(y) + step_y;
                if (side_x >= 0 && side_x < static_cast<std::int64_t>(grid.width))
                {
                    const auto side_x_index = static_cast<std::size_t>(side_x);
                    if (!update_cell(side_x_index, y, next_label))
                        return false;
                    endpoint_updated = side_x_index == end_x && y == end_y;
                }
                if (side_y >= 0 && side_y < static_cast<std::int64_t>(grid.height))
                {
                    const auto side_y_index = static_cast<std::size_t>(side_y);
                    if (!update_cell(x, side_y_index, next_label))
                        return false;
                    endpoint_updated = endpoint_updated || (x == end_x && side_y_index == end_y);
                }
            }
            if (!endpoint_updated && !update_cell(end_x, end_y, next_label))
                return false;
            return true;
        }

        BOOST_ASSERT(crosses_x || crosses_y);
        if (crosses_x && crosses_y)
        {
            // The segment passes exactly through a grid corner. Mark both cells that touch
            // the crossing so a continuous polyline stays four-connected when binary cell
            // contours are built. Independently occupied diagonal cells remain disconnected.
            const auto side_x = static_cast<std::int64_t>(x) + step_x;
            const auto side_y = static_cast<std::int64_t>(y) + step_y;
            BOOST_ASSERT(side_x >= 0);
            BOOST_ASSERT(side_x < static_cast<std::int64_t>(grid.width));
            BOOST_ASSERT(side_y >= 0);
            BOOST_ASSERT(side_y < static_cast<std::int64_t>(grid.height));
            if (!update_cell(static_cast<std::size_t>(side_x), y, next_label) ||
                !update_cell(x, static_cast<std::size_t>(side_y), next_label))
            {
                return false;
            }
        }
        if (crosses_x)
        {
            x = static_cast<std::size_t>(static_cast<std::int64_t>(x) + step_x);
            t_max_x += t_delta_x;
        }
        if (crosses_y)
        {
            y = static_cast<std::size_t>(static_cast<std::int64_t>(y) + step_y);
            t_max_y += t_delta_y;
        }
        t = next_t;
    }

    BOOST_ASSERT(false);
    return false;
}

util::Coordinate coordinateAt(const WeightedGrid &grid, const GridPoint point)
{
    const auto projection = makeProjection(grid);
    const auto x = grid.origin_x + point.x * grid.cell_size;
    const auto y = grid.origin_y + point.y * grid.cell_size;
    return {util::FloatLongitude{projection.longitudeAt(x)},
            util::FloatLatitude{projection.latitudeAt(y)}};
}

CoordinateRing makeCoordinateRing(const WeightedGrid &grid, const GridRing &ring)
{
    CoordinateRing coordinates;
    coordinates.reserve(ring.size());
    for (const auto point : ring)
        coordinates.push_back(coordinateAt(grid, point));
    return coordinates;
}

} // namespace

std::vector<CoordinatePolygon> WeightedGrid::buildContours(const double cutoff) const
{
    auto contours = buildContours(cutoff, std::numeric_limits<std::size_t>::max());
    BOOST_ASSERT(contours);
    return std::move(*contours);
}

std::optional<std::vector<CoordinatePolygon>>
WeightedGrid::buildContours(const double cutoff, const std::size_t maximum_coordinates) const
{ return buildContours(cutoff, maximum_coordinates, 0.); }

std::optional<std::vector<CoordinatePolygon>>
WeightedGrid::buildContours(const double cutoff,
                            const std::size_t maximum_coordinates,
                            const double generalize_metres) const
{
    std::size_t raw_coordinate_count = 0;
    return buildContours(cutoff, maximum_coordinates, generalize_metres, 0., raw_coordinate_count);
}

std::optional<std::vector<CoordinatePolygon>>
WeightedGrid::buildContours(const double cutoff,
                            const std::size_t maximum_coordinates,
                            const double generalize_metres,
                            const double denoise) const
{
    std::size_t raw_coordinate_count = 0;
    return buildContours(
        cutoff, maximum_coordinates, generalize_metres, denoise, raw_coordinate_count);
}

std::optional<std::vector<CoordinatePolygon>>
WeightedGrid::buildContours(const double cutoff,
                            const std::size_t maximum_coordinates,
                            const double generalize_metres,
                            std::size_t &raw_coordinate_count) const
{ return buildContours(cutoff, maximum_coordinates, generalize_metres, 0., raw_coordinate_count); }

std::optional<std::vector<CoordinatePolygon>>
WeightedGrid::buildContours(const double cutoff,
                            const std::size_t maximum_coordinates,
                            const double generalize_metres,
                            const double denoise,
                            std::size_t &raw_coordinate_count) const
{
    BOOST_ASSERT(width == 0 || height <= std::numeric_limits<std::size_t>::max() / width);
    BOOST_ASSERT(values.size() == width * height);

    auto grid_polygons = buildCellContours(values, width, height, cutoff, maximum_coordinates);
    if (!grid_polygons)
        return std::nullopt;

    raw_coordinate_count = 0;
    for (const auto &polygon : *grid_polygons)
    {
        raw_coordinate_count += polygon.outer.size();
        for (const auto &hole : polygon.holes)
            raw_coordinate_count += hole.size();
    }

    if (denoise > 0.)
    {
        const auto denoised = denoiseGridPolygons(*grid_polygons, denoise);
        BOOST_ASSERT(denoised);
        static_cast<void>(denoised);
    }

    if (generalize_metres > 0. && cell_size > 0.)
    {
        const auto grid_tolerance = generalize_metres / cell_size;
        generalizeGridPolygons(*grid_polygons,
                               std::isfinite(grid_tolerance) ? grid_tolerance
                                                             : std::numeric_limits<double>::max());
    }

    std::vector<CoordinatePolygon> polygons;
    polygons.reserve(grid_polygons->size());
    for (const auto &grid_polygon : *grid_polygons)
    {
        CoordinatePolygon polygon{makeCoordinateRing(*this, grid_polygon.outer), {}};
        polygon.holes.reserve(grid_polygon.holes.size());
        for (const auto &hole : grid_polygon.holes)
            polygon.holes.push_back(makeCoordinateRing(*this, hole));
        polygons.push_back(std::move(polygon));
    }
    return polygons;
}

RasterizationResult
rasterizeWeightedPolylines(const std::span<const WeightedPolyline> polylines,
                           const WeightedGridOptions &options,
                           const std::optional<util::Coordinate> projection_center)
{
    if (!std::isfinite(options.cell_size) || options.cell_size < MINIMUM_CELL_SIZE_METRES ||
        options.maximum_cell_count == 0 || options.maximum_rasterization_steps == 0 ||
        !(options.maximum_duration >= 0.))
    {
        return {{}, RasterizationError::InvalidOptions};
    }

    std::vector<PolylineSegment> segments;
    std::vector<SelectedSegment> selected;
    const auto preparation_error = prepareSegments(polylines, options, segments, selected);
    if (preparation_error != RasterizationError::None)
        return {{}, preparation_error};
    const auto endpoints = [&](const SelectedSegment &segment)
    {
        auto from = pointAt(segments[segment.segment], segment.interval.begin);
        auto to = pointAt(segments[segment.segment], segment.interval.end);
        from.duration = std::min(from.duration, options.maximum_duration);
        to.duration = std::min(to.duration, options.maximum_duration);
        if (!segment.interval.include_begin)
            from.duration = std::nextafter(from.duration, to.duration);
        if (!segment.interval.include_end)
            to.duration = std::nextafter(to.duration, from.duration);
        return std::pair{from, to};
    };

    Bounds bounds;
    auto found_point = false;
    for (const auto segment : selected)
    {
        const auto [from, to] = endpoints(segment);
        for (const auto &point : {from, to})
        {
            const auto lon = longitude(point.coordinate);
            const auto lat = latitude(point.coordinate);
            if (std::abs(lat) >= 90.)
                return {{}, RasterizationError::TouchesPole};
            bounds.min_lon = std::min(bounds.min_lon, lon);
            bounds.max_lon = std::max(bounds.max_lon, lon);
            bounds.min_lat = std::min(bounds.min_lat, lat);
            bounds.max_lat = std::max(bounds.max_lat, lat);
            found_point = true;
        }
    }

    if (!found_point)
        return {WeightedGrid{}, RasterizationError::None};

    // A single local plane cannot represent input that crosses the
    // antimeridian without longitude unwrapping and RFC 7946 splitting.
    if (bounds.max_lon - bounds.min_lon >= 180.)
        return {{}, RasterizationError::TouchesLongitudeBoundary};

    const auto fallback_center =
        util::Coordinate{util::FloatLongitude{(bounds.min_lon + bounds.max_lon) / 2.},
                         util::FloatLatitude{(bounds.min_lat + bounds.max_lat) / 2.}};
    const auto center = projection_center.value_or(fallback_center);
    if (!center.IsValid())
        return {{}, RasterizationError::InvalidOptions};
    if (std::abs(latitude(center)) >= 90.)
        return {{}, RasterizationError::TouchesPole};

    const auto projection = makeProjection(center);
    MetricBounds metric_bounds;
    for (const auto segment : selected)
    {
        const auto [from, to] = endpoints(segment);
        for (const auto &point : {from, to})
        {
            if (std::abs(longitude(point.coordinate) - projection.reference_lon) >= 180.)
                return {{}, RasterizationError::TouchesLongitudeBoundary};
            const auto projected = projection.project(point.coordinate);
            metric_bounds.min_x = std::min(metric_bounds.min_x, projected.x);
            metric_bounds.max_x = std::max(metric_bounds.max_x, projected.x);
            metric_bounds.min_y = std::min(metric_bounds.min_y, projected.y);
            metric_bounds.max_y = std::max(metric_bounds.max_y, projected.y);
        }
    }

    const auto bounds_in_cells = gridBounds(metric_bounds, options.cell_size);
    if (!bounds_in_cells)
        return {{}, RasterizationError::TooBig};

    const auto extent_error =
        rasterizationExtentError(*bounds_in_cells, options.cell_size, projection);
    if (extent_error != RasterizationError::None)
        return {{}, extent_error};

    if (exceedsCellCap(*bounds_in_cells, options.maximum_cell_count))
        return {{}, RasterizationError::TooBig};

    const auto width = dimension(bounds_in_cells->min_x, bounds_in_cells->max_x);
    const auto height = dimension(bounds_in_cells->min_y, bounds_in_cells->max_y);
    BOOST_ASSERT(width && height);

    WeightedGrid grid{
        util::FloatLongitude{projection.reference_lon},
        util::FloatLatitude{projection.reference_lat},
        options.cell_size,
        *width,
        *height,
        std::vector<double>(*width * *height, std::numeric_limits<double>::infinity()),
        static_cast<double>(static_cast<long double>(bounds_in_cells->min_x) * options.cell_size),
        static_cast<double>(static_cast<long double>(bounds_in_cells->min_y) * options.cell_size)};

    std::size_t rasterization_steps = 0;
    const auto update_cell = [&](const std::size_t x, const std::size_t y, const double duration)
    {
        BOOST_ASSERT(x < grid.width);
        BOOST_ASSERT(y < grid.height);
        if (rasterization_steps >= options.maximum_rasterization_steps)
            return false;
        const auto index = y * grid.width + x;
        grid.values[index] = std::min(grid.values[index], duration);
        ++rasterization_steps;
        return true;
    };
    for (const auto segment : selected)
    {
        const auto [from, to] = endpoints(segment);
        if (!rasterizeSegment(grid,
                              from,
                              to,
                              segment.interval.include_begin,
                              segment.interval.include_end,
                              update_cell))
            return {{}, RasterizationError::TooBig};
    }
    return {std::move(grid), RasterizationError::None};
}

} // namespace osrm::engine::isochrone
