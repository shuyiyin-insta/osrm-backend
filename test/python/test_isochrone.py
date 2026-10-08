import constants
import pytest

import osrm

mld_data_path = constants.mld_data_path
one_test_coordinate = constants.two_test_coordinates[0]


class TestIsochrone:
    osrm_py = osrm.OSRM(
        storage_config=mld_data_path,
        algorithm="MLD",
        use_shared_memory=False,
    )

    def test_parameters(self):
        params = osrm.IsochroneParameters(
            coordinates=[one_test_coordinate],
            contours_seconds=[300.0, 600.0],
            direction="inbound",
            polygons=False,
            generalize=25.0,
            denoise=0.25,
        )
        assert params.IsValid()
        assert params.contours_seconds == [300.0, 600.0]
        assert not hasattr(params, "contours")
        assert repr(params.direction) == "inbound"
        assert not params.polygons
        assert params.generalize == 25.0
        assert params.denoise == 0.25

        params.generalize = 0.0
        assert params.IsValid()
        assert params.generalize == 0.0

        params.generalize = None
        assert params.IsValid()
        assert params.generalize is None

        params.denoise = 0.0
        assert params.IsValid()
        assert params.denoise == 0.0

        params.denoise = None
        assert params.IsValid()
        assert params.denoise is None

    def test_returns_a_geojson_feature_collection(self):
        params = osrm.IsochroneParameters(
            coordinates=[one_test_coordinate],
            contours_seconds=[300.0, 600.0],
            polygons=False,
            generalize=25.0,
            denoise=0.1,
        )
        result = self.osrm_py.Isochrone(params)

        assert result["code"] == "Ok"
        assert result["type"] == "FeatureCollection"
        assert "weight_name" not in result
        assert len(result["features"]) == 2
        assert all(
            feature["type"] == "Feature"
            and feature["geometry"]["type"] == "MultiLineString"
            and isinstance(feature["geometry"]["coordinates"], osrm.Array)
            and feature["properties"]["effective_contour_seconds"]
            <= feature["properties"]["contour_seconds"]
            for feature in result["features"]
        )

    def test_documented_constructor_and_direction_assignment(self):
        params = osrm.IsochroneParameters(
            coordinates=[(7.41337, 43.72956)],
            contours_seconds=[300.0, 600.0],
            direction="outbound",
        )
        assert params.IsValid()

        params.direction = "inbound"
        assert params.IsValid()
        assert repr(params.direction) == "inbound"

        enum_params = osrm.IsochroneParameters(
            coordinates=[osrm.Coordinate((7.41337, 43.72956))],
            contours_seconds=[300.0],
            direction=params.direction,
        )
        assert enum_params.IsValid()
        assert repr(enum_params.direction) == "inbound"

    def test_reports_the_decisecond_duration_contour_that_was_evaluated(self):
        params = osrm.IsochroneParameters(
            coordinates=[one_test_coordinate],
            contours_seconds=[0.15],
        )
        result = self.osrm_py.Isochrone(params)

        assert result["code"] == "Ok"
        assert result["features"][0]["properties"]["effective_contour_seconds"] == 0.1

    def test_rejects_invalid_parameters_before_querying(self):
        params = osrm.IsochroneParameters(coordinates=[one_test_coordinate])
        with pytest.raises(RuntimeError, match="Invalid Isochrone Parameters"):
            self.osrm_py.Isochrone(params)

    def test_rejects_legacy_contours_argument(self):
        with pytest.raises(TypeError):
            osrm.IsochroneParameters(
                coordinates=[one_test_coordinate],
                contours=[300.0],
            )

    @pytest.mark.parametrize("generalize", [-1.0, float("inf"), float("nan")])
    def test_rejects_invalid_generalize_before_querying(self, generalize):
        params = osrm.IsochroneParameters(
            coordinates=[one_test_coordinate],
            contours_seconds=[300.0],
            generalize=generalize,
        )
        with pytest.raises(RuntimeError, match="Invalid Isochrone Parameters"):
            self.osrm_py.Isochrone(params)

    def test_rejects_non_numeric_generalize(self):
        with pytest.raises(TypeError):
            osrm.IsochroneParameters(
                coordinates=[one_test_coordinate],
                contours_seconds=[300.0],
                generalize="25",
            )

    @pytest.mark.parametrize("denoise", [-0.1, 1.1, float("inf"), float("nan")])
    def test_rejects_invalid_denoise_before_querying(self, denoise):
        params = osrm.IsochroneParameters(
            coordinates=[one_test_coordinate],
            contours_seconds=[300.0],
            denoise=denoise,
        )
        with pytest.raises(RuntimeError, match="Invalid Isochrone Parameters"):
            self.osrm_py.Isochrone(params)

    def test_rejects_non_numeric_denoise(self):
        with pytest.raises(TypeError):
            osrm.IsochroneParameters(
                coordinates=[one_test_coordinate],
                contours_seconds=[300.0],
                denoise="0.25",
            )


@pytest.mark.parametrize("algorithm,path", [("CH", constants.data_path), ("MLD", mld_data_path)])
def test_search_node_limit_does_not_affect_route_or_table(algorithm, path):
    engine = osrm.OSRM(
        storage_config=path,
        algorithm=algorithm,
        use_shared_memory=False,
        max_isochrone_search_nodes=1,
    )
    params = osrm.IsochroneParameters(
        coordinates=[one_test_coordinate],
        contours_seconds=[300.0],
    )
    for _ in range(2):
        with pytest.raises(
            RuntimeError, match="TooBig - Isochrone search exceeds the configured node limit"
        ):
            engine.Isochrone(params)
        assert (
            engine.Route(osrm.RouteParameters(coordinates=constants.two_test_coordinates))["code"]
            == "Ok"
        )
        assert (
            engine.Table(osrm.TableParameters(coordinates=constants.two_test_coordinates))["code"]
            == "Ok"
        )


@pytest.mark.parametrize("algorithm,path", [("CH", constants.data_path), ("MLD", mld_data_path)])
@pytest.mark.parametrize("limit", [-1, 10_000_000])
def test_accepts_unlimited_and_large_search_node_limits(algorithm, path, limit):
    config = osrm.EngineConfig(
        storage_config=path,
        algorithm=algorithm,
        use_shared_memory=False,
        max_isochrone_search_nodes=limit,
    )
    assert config.max_isochrone_search_nodes == limit
    result = osrm.OSRM(config).Isochrone(
        osrm.IsochroneParameters(coordinates=[one_test_coordinate], contours_seconds=[300.0])
    )
    assert result["code"] == "Ok"


def test_search_node_limit_default_and_property_validation():
    config = osrm.EngineConfig()
    assert config.max_isochrone_search_nodes == -1
    for limit in [-1, 1, 2_147_483_647]:
        config.max_isochrone_search_nodes = limit
        assert config.IsValid()
    for limit in [0, -2, -2_147_483_648]:
        config.max_isochrone_search_nodes = limit
        assert not config.IsValid()


@pytest.mark.parametrize("limit", [0, -2, -2_147_483_648])
def test_rejects_invalid_search_node_limits(limit):
    with pytest.raises(RuntimeError, match="Config Parameters are Invalid"):
        osrm.EngineConfig(max_isochrone_search_nodes=limit)
    with pytest.raises(RuntimeError, match="Config Parameters are Invalid"):
        osrm.OSRM(max_isochrone_search_nodes=limit)


@pytest.mark.parametrize("limit", ["1", None, 1.5, float("inf"), float("nan"), 2_147_483_648])
def test_rejects_non_integer_or_out_of_range_search_node_limits(limit):
    with pytest.raises(
        RuntimeError, match="Invalid type passed for argument: max_isochrone_search_nodes"
    ):
        osrm.EngineConfig(max_isochrone_search_nodes=limit)
    with pytest.raises(
        RuntimeError, match="Invalid type passed for argument: max_isochrone_search_nodes"
    ):
        osrm.OSRM(max_isochrone_search_nodes=limit)
    with pytest.raises(TypeError):
        osrm.EngineConfig().max_isochrone_search_nodes = limit
