//
// Created by lutz on 02.09.22.
//

#ifndef TGLIB_TEMPORALDIAMETER_H
#define TGLIB_TEMPORALDIAMETER_H

#include <vector>
#include "../core/BasicTypes.h"
#include "TemporalDistances.h"

namespace tglib {

/**
 * @brief Computes the maximum optimal temporal distance to any reachable vertex
 *
 * @tparam TempGraph The temporal graph type
 * @param tg The temporal graph
 * @param nid The node id
 * @param timeInterval The restrictive time interval
 * @param distanceType The distance time
 * @return The eccentricity of node nid
 */
template<typename TempGraph>
Time temporal_eccentricity(TempGraph const &tg, NodeId nid, tglib::TimeInterval timeInterval, tglib::Distance_Type distanceType) {
    std::vector<Time> distances;
    switch (distanceType) {

        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times(tg, nid, timeInterval);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations(tg, nid, timeInterval);
            break;
      case Distance_Type::Latest_Departure:
//     todo       distances = latest_departure_times(tg, nid, timeInterval);
            return inf;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times(tg, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Hops:
            distances = minimum_hops(tg, nid, timeInterval);
            break;
    }

    Time max_d = 0;
    for (auto const &d : distances) {
        if (d < inf && d > max_d)
            max_d = d;
    }
    return max_d;
}


/**
 * @brief Computes the temporal diameter, e.g., max. eccentricity in the temporal graph
 *
 * @tparam TempGraph The temporal graph type
 * @param tg The temporal graph
 * @param timeInterval The restrictive time interval
 * @param distanceType The distance time
 * @return The temporal diameter of tg
 */
template<typename TempGraph>
Time temporal_diameter(TempGraph const &tg, tglib::TimeInterval timeInterval, tglib::Distance_Type distanceType) {
    std::vector<Time> ecc(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, ecc, distanceType, timeInterval)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        ecc[nid] = temporal_eccentricity(tg, nid, timeInterval, distanceType);
    }

    Time diameter = 0;
    for (auto &e: ecc) {
        if (e < inf && e > diameter)
            diameter = e;
    }

    return diameter;
}

} // tglib

#endif //TGLIB_TEMPORALDIAMETER_H
