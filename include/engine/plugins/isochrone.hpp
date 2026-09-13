#ifndef ISOCHRONEPLUGIN_HPP
#define ISOCHRONEPLUGIN_HPP

#include "engine/api/isochrone_parameters.hpp"
#include "engine/plugins/plugin_base.hpp"
#include <cstddef>
#include <optional>

namespace osrm::engine::plugins
{

class IsochronePlugin final : public BasePlugin
{
  public:
    IsochronePlugin(const std::optional<double> default_radius, const int max_range_seconds)
        : BasePlugin(default_radius), max_range_seconds(max_range_seconds)
    {
    }

    Status HandleRequest(const RoutingAlgorithmsInterface &algorithms,
                         const api::IsochroneParameters &parameters,
                         osrm::engine::api::ResultT &result) const;

  private:
    static constexpr std::size_t MAX_SEARCH_RECORDS = 100'000;
    static constexpr std::size_t MAX_MATERIALIZED_POINTS = 1'000'000;
    static constexpr std::size_t MAX_RASTERIZATION_STEPS = 5'000'000;
    static constexpr std::size_t MAX_OUTPUT_POINTS = 1'000'000;
    static constexpr std::size_t MAX_GRID_CELLS = 250'000;
    static constexpr std::size_t MAX_CONTOURS = 10;

    const int max_range_seconds;
};

} // namespace osrm::engine::plugins

#endif // ISOCHRONEPLUGIN_HPP
