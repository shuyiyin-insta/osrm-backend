#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"
#include "engine/datafacade/process_memory_allocator.hpp"

#include "extractor/profile_properties.hpp"
#include "storage/storage_config.hpp"

#include <boost/test/unit_test.hpp>

#include <memory>

namespace osrm::engine::datafacade
{

BOOST_AUTO_TEST_SUITE(isochrone_ch_topological_order)

BOOST_AUTO_TEST_CASE(caches_the_filtered_monaco_ch_graph)
{
    const auto allocator = std::make_shared<ProcessMemoryAllocator>(
        storage::StorageConfig(OSRM_TEST_DATA_DIR "/ch/monaco.osrm"));
    const auto *properties =
        allocator->GetIndex().GetBlockPtr<extractor::ProfileProperties>("/common/properties");
    const ContiguousInternalMemoryDataFacade<CH> facade{
        allocator, properties->GetWeightName(), 0};

    const auto *order = facade.GetIsochroneTopologicalOrder();
    const auto *cached_order = facade.GetIsochroneTopologicalOrder();

    BOOST_REQUIRE(order != nullptr);
    BOOST_CHECK_EQUAL(order, cached_order);
    BOOST_CHECK_EQUAL(order->rank.size(), facade.GetNumberOfNodes());
    BOOST_CHECK_LT(order->nodes.size(), facade.GetNumberOfNodes());

    bool has_uncontracted_core = false;
    for (const auto rank : order->rank)
        has_uncontracted_core = has_uncontracted_core || rank == order->nodes.size();
    BOOST_CHECK(has_uncontracted_core);
}

BOOST_AUTO_TEST_SUITE_END()

} // namespace osrm::engine::datafacade
