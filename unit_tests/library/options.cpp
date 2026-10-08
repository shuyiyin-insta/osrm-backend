#include <boost/test/unit_test.hpp>

#include "coordinates.hpp"
#include "equal_json.hpp"
#include "fixture.hpp"

#include "osrm/osrm.hpp"

#include <limits>

BOOST_AUTO_TEST_SUITE(options)

BOOST_AUTO_TEST_CASE(test_ch)
{
    using namespace osrm;
    EngineConfig config;
    config.use_shared_memory = false;
    config.storage_config = storage::StorageConfig(OSRM_TEST_DATA_DIR "/ch/monaco.osrm");
    config.algorithm = EngineConfig::Algorithm::CH;
    OSRM osrm{config};
}

BOOST_AUTO_TEST_CASE(test_mld)
{
    using namespace osrm;
    EngineConfig config;
    config.use_shared_memory = false;
    config.storage_config = storage::StorageConfig(OSRM_TEST_DATA_DIR "/mld/monaco.osrm");
    config.algorithm = EngineConfig::Algorithm::MLD;
    OSRM osrm{config};
}

BOOST_AUTO_TEST_CASE(test_isochrone_search_node_limits)
{
    osrm::EngineConfig config;
    BOOST_CHECK_EQUAL(config.max_isochrone_search_nodes, -1);
    BOOST_CHECK(config.IsValid());
    for (const auto limit : {-1, 1, 10'000'000, std::numeric_limits<int>::max()})
    {
        config.max_isochrone_search_nodes = limit;
        BOOST_CHECK(config.IsValid());
    }
    for (const auto limit : {0, -2, std::numeric_limits<int>::min()})
    {
        config.max_isochrone_search_nodes = limit;
        BOOST_CHECK(!config.IsValid());
    }
}

BOOST_AUTO_TEST_SUITE_END()
