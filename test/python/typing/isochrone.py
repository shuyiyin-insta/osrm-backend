from osrm.osrm_ext import IsochroneDirection

import osrm


def documented_parameters() -> osrm.IsochroneParameters:
    params = osrm.IsochroneParameters(
        coordinates=[(7.41337, 43.72956)],
        contours_seconds=[300.0, 600.0],
        direction="outbound",
        snapping="default",
    )
    update_direction(params)
    return params


def update_direction(params: osrm.IsochroneParameters) -> IsochroneDirection:
    params.direction = "inbound"
    params.direction = IsochroneDirection("outbound")
    return params.direction


def coordinate_and_direction_objects() -> osrm.IsochroneParameters:
    params = documented_parameters()
    return osrm.IsochroneParameters(
        coordinates=[osrm.Coordinate((7.41337, 43.72956))],
        contours_seconds=[300.0],
        direction=params.direction,
    )
