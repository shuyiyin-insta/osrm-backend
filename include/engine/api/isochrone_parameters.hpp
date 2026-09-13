/*

Copyright (c) 2016, Project OSRM contributors
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef ENGINE_API_ISOCHRONE_PARAMETERS_HPP
#define ENGINE_API_ISOCHRONE_PARAMETERS_HPP

#include "engine/api/base_parameters.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <vector>

namespace osrm
{
namespace engine
{
namespace api
{

/**
 * Parameters specific to the OSRM Isochrone service.
 *
 * The contours_seconds member contains elapsed-duration thresholds in seconds.
 */
struct IsochroneParameters : public BaseParameters
{
    enum class Direction
    {
        Outbound,
        Inbound
    };

    std::vector<double> contours_seconds;
    Direction direction = Direction::Outbound;
    bool polygons = true;
    std::optional<double> generalize = std::nullopt;
    std::optional<double> denoise = std::nullopt;

    // Retained while the existing max_isochrone_range configuration is wired into the engine.
    unsigned range = 15 * 60;

    bool operator==(const IsochroneParameters &) const = default;

    bool IsValid() const
    {
        return BaseParameters::IsValid() && coordinates.size() == 1 && !contours_seconds.empty() &&
               (direction == Direction::Outbound || direction == Direction::Inbound) &&
               (!generalize || (std::isfinite(*generalize) && *generalize >= 0.)) &&
               (!denoise || (std::isfinite(*denoise) && *denoise >= 0. && *denoise <= 1.)) &&
               std::all_of(contours_seconds.begin(),
                           contours_seconds.end(),
                           [](const double contour_seconds)
                           { return std::isfinite(contour_seconds) && contour_seconds > 0.; });
    }
};
} // namespace api
} // namespace engine
} // namespace osrm

#endif
