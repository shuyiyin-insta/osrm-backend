#ifndef OSRM_ENGINE_ISOCHRONE_NATIVE_SEARCH_RESULT_HPP
#define OSRM_ENGINE_ISOCHRONE_NATIVE_SEARCH_RESULT_HPP

#include "engine/datafacade/datafacade_base.hpp"
#include "engine/isochrone/search_result.hpp"
#include "engine/phantom_node.hpp"
#include "engine/routing_algorithms/reachability.hpp"

#include <cstddef>

namespace osrm::engine::isochrone
{

// Converts native CH/MLD one-to-all labels into the geometry-scoped contract consumed by the
// contour materializer. The native searches retain every settled minimum-weight label; only
// labels that can affect a geometry intersecting the duration cutoff are carried forward.
SearchResult makeNativeSearchResult(
    const datafacade::BaseDataFacade &facade,
    const routing_algorithms::ReachabilitySearchResult &native_result,
    const PhantomNodeCandidates &endpoint_candidates,
    EdgeDuration duration_cutoff,
    std::size_t maximum_records,
    bool inbound);

} // namespace osrm::engine::isochrone

#endif // OSRM_ENGINE_ISOCHRONE_NATIVE_SEARCH_RESULT_HPP
