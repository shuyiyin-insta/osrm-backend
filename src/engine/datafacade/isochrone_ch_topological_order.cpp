#include "engine/datafacade/contiguous_internalmem_datafacade.hpp"

namespace osrm::engine::datafacade
{

const routing_algorithms::ch::IsochroneCHTopologicalOrder *
ContiguousInternalMemoryAlgorithmDataFacade<CH>::GetIsochroneTopologicalOrder() const
{
    std::call_once(m_isochrone_topological_order_once,
                   [this]
                   {
                       routing_algorithms::ch::IsochroneCHTopologicalOrder order;
                       if (routing_algorithms::ch::buildIsochroneCHTopologicalOrder(*this, order))
                           m_isochrone_topological_order = std::move(order);
                   });

    return m_isochrone_topological_order ? &*m_isochrone_topological_order : nullptr;
}

} // namespace osrm::engine::datafacade
