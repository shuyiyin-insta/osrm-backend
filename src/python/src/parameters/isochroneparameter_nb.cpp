#include "python/parameters/isochroneparameter_nb.hpp"

#include "python/parameters/baseparameter_nb.hpp"
#include "python/types/approach_nb.hpp"
#include "python/utility/param_utility.hpp"

#include <nanobind/nanobind.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

namespace nb = nanobind;

void init_IsochroneParameters(nb::module_ &m)
{
    using osrm::engine::api::BaseParameters;
    using osrm::engine::api::IsochroneParameters;

    nb::class_<IsochroneParameters, BaseParameters>(m, "IsochroneParameters")
        .def(nb::init<>())
        .def(
            "__init__",
            [](IsochroneParameters *t,
               std::vector<double> contours_seconds,
               IsochroneParameters::Direction direction,
               bool polygons,
               std::vector<osrm::util::Coordinate> coordinates,
               std::vector<std::optional<std::string>> hints,
               std::vector<std::optional<double>> radiuses,
               std::vector<std::optional<osrm::engine::Bearing>> bearings,
               const std::vector<std::optional<osrm::engine::Approach>> &approaches,
               bool generate_hints,
               std::vector<std::string> exclude,
               const BaseParameters::SnappingType snapping,
               std::optional<double> generalize,
               std::optional<double> denoise)
            {
                new (t) IsochroneParameters();

                t->contours_seconds = std::move(contours_seconds);
                t->direction = direction;
                t->polygons = polygons;
                t->generalize = generalize;
                t->denoise = denoise;

                osrm_nb_util::assign_baseparameters(t,
                                                    coordinates,
                                                    hints,
                                                    radiuses,
                                                    bearings,
                                                    approaches,
                                                    generate_hints,
                                                    exclude,
                                                    snapping);
            },
            nb::arg("contours_seconds") = std::vector<double>(),
            nb::arg("direction") = std::string(),
            nb::arg("polygons") = true,
            nb::arg("coordinates") = std::vector<osrm::util::Coordinate>(),
            nb::arg("hints") = std::vector<std::optional<std::string>>(),
            nb::arg("radiuses") = std::vector<std::optional<double>>(),
            nb::arg("bearings") = std::vector<std::optional<osrm::engine::Bearing>>(),
            nb::arg("approaches") = std::vector<std::optional<osrm::engine::Approach>>(),
            nb::arg("generate_hints") = true,
            nb::arg("exclude") = std::vector<std::string>(),
            nb::arg("snapping") = std::string(),
            nb::arg("generalize") = std::optional<double>(),
            nb::arg("denoise") = std::optional<double>())
        .def_rw("contours_seconds", &IsochroneParameters::contours_seconds)
        .def_rw("direction", &IsochroneParameters::direction)
        .def_rw("polygons", &IsochroneParameters::polygons)
        .def_rw("generalize", &IsochroneParameters::generalize)
        .def_rw("denoise", &IsochroneParameters::denoise)
        .def("IsValid", &IsochroneParameters::IsValid);

    nb::class_<IsochroneParameters::Direction>(m, "IsochroneDirection")
        .def("__init__",
             [](IsochroneParameters::Direction *t, const std::string &str)
             {
                 const auto direction =
                     osrm_nb_util::str_to_enum(str, "IsochroneDirection", isochrone_direction_map);
                 new (t) IsochroneParameters::Direction(direction);
             })
        .def("__repr__",
             [](IsochroneParameters::Direction direction)
             {
                 return osrm_nb_util::enum_to_str(
                     direction, "IsochroneDirection", isochrone_direction_map);
             });
    nb::implicitly_convertible<std::string, IsochroneParameters::Direction>();
}
