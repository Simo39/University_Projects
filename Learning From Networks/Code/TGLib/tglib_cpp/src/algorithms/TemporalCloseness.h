/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalCloseness.h
 *  @brief Contains function declarations for computing closeness centrality
 */

#ifndef TGLIB_TEMPORALCLOSENESS_H
#define TGLIB_TEMPORALCLOSENESS_H


#include "../core/OrderedEdgeList.h"
#include "../core/IncidentLists.h"
#include "../core/TRSGraph.h"
#include "../algorithms/TemporalDistances.h"

namespace tglib {

/**
 * @brief Computes the closeness of a single node
 *
 * @param tgs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param timeInterval The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename E>
double temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                         tglib::NodeId nid,
                         tglib::TimeInterval timeInterval,
                         tglib::Distance_Type distanceType) {
    std::vector<Time> distances;

    switch (distanceType) {

        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times<E>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations(tgs, nid, timeInterval);
            break;
        case Distance_Type::Latest_Departure:
            distances = latest_departure_times(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Hops:
            distances = minimum_hops(tgs, nid, timeInterval);
            break;
    }

    double closeness = 0;

    for (long distance : distances) {
        if (distance > 0 && distance < inf) {
            closeness += 1.0 / (double)distance;
        }
    }

    return closeness;
}

/**
 * @brief Computes the closeness of a single node
 *
 * @param tgs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param timeInterval The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename TempGraph>
double run_temporal_closeness(TempGraph const &tgs,
                         tglib::NodeId nid,
                         tglib::TimeInterval timeInterval,
                         tglib::Distance_Type distanceType) {
    std::vector<Time> distances;

    switch (distanceType) {

        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Latest_Departure:
            distances = latest_departure_times<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times<TempGraph>(tgs, nid, timeInterval);
            break;
        case Distance_Type::Minimum_Hops:
            distances = minimum_hops<TempGraph>(tgs, nid, timeInterval);
            break;
    }

    double closeness = 0;

    for (long distance : distances) {
        if (distance > 0 && distance < inf) {
            closeness += 1.0 / (double)distance;
        }
    }

    return closeness;
}

/**
 * @brief Computes the closeness of a single node
 *
 * @param tgs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename E>
double temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                                 tglib::NodeId nid,
                                 tglib::Distance_Type distanceType) {
    return temporal_closeness(tgs, nid, tgs.getTimeInterval(), distanceType);
}

/**
 * Computes the closeness of a single node
 * @param tg The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param ti The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness of the node with id nid.
 */
template<typename N, typename E>
double temporal_closeness(tglib::IncidentLists<N, E> const &tg,
                         tglib::NodeId nid,
                         tglib::TimeInterval ti,
                         tglib::Distance_Type distanceType) {
    std::vector<Time> distances;
    switch (distanceType) {
        case Distance_Type::Earliest_Arrival:
            distances = earliest_arrival_times(tg, nid, ti);
            break;
        case Distance_Type::Fastest:
            distances = minimum_durations(tg, nid, ti);
            break;
        case Distance_Type::Latest_Departure:
// todo           latest_departure_times(tg, nid, ti);
            break;
        case Distance_Type::Minimum_Transition_Times:
            distances = minimum_transition_times(tg, nid, ti);
            break;
        case Distance_Type::Minimum_Hops:
// todo           distances = minimum_hops(tg, nid, ti);
            break;
    }
    double closeness = 0;
    for (auto &d : distances) {
        if (d > 0 && d < inf) {
            closeness += 1.0 / (double)d;
        }
    }
    return closeness;
}

/**
 * Computes the closeness of a single node
 * @param trs The temporal graph.
 * @param nid The node for which the closeness is computed.
 * @param ti The restrictive time interval.
 * @return The temporal closeness of the node with id nid.
 */
inline double temporal_closeness(tglib::TRSGraph &trs,
                         tglib::NodeId nid,
                         tglib::TimeInterval ti) {
    auto durations = minimum_durations(trs, nid, ti);
    double closeness = 0;
    for (auto &d : durations) {
        if (d > 0 && d < inf) {
            closeness += 1.0 / (double)d;
        }
    }
    return closeness;
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tgs The temporal graph.
 * @param timeInterval The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness values of all nodes
 */
template<typename E>
std::vector<double> temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                                      tglib::TimeInterval timeInterval,
                                      tglib::Distance_Type distanceType) {
    std::vector<double> closeness_values(tgs.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tgs, timeInterval, closeness_values, distanceType)
    for (size_t i = 0; i < tgs.getNumberOfNodes(); ++i) {
        auto c = temporal_closeness(tgs, i, timeInterval, distanceType);
        closeness_values[i] = c;
    }

    return closeness_values;
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tgs The temporal graph.
 * @param distanceType The distance type.
 * @return The temporal closeness values of all nodes
 */
template<typename E>
std::vector<double> temporal_closeness(tglib::OrderedEdgeList<E> const &tgs,
                                      tglib::Distance_Type distanceType) {
    return temporal_closeness(tgs, tgs.getTimeInterval() , distanceType);
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tg The temporal graph.
 * @param ti The restrictive time interval.
 * @param distanceType The distance type.
 * @return The temporal closeness values of all nodes
 */
template<typename N, typename E>
std::vector<double> temporal_closeness(tglib::IncidentLists<N, E>&tg, tglib::TimeInterval ti,
                                      tglib::Distance_Type distanceType) {
    std::vector<double> closeness(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, ti, closeness, distanceType)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        closeness[nid] = temporal_closeness(tg, (NodeId)nid, ti, distanceType);
    }

    return closeness;
}

/**
 * @brief Computes the closeness of all nodes
 *
 * @param tg The temporal graph.
 * @param distanceType The distance type
 * @return The temporal closeness values of all nodes
 */
template<typename TempGraph>
std::vector<double> run_temporal_closeness(TempGraph const& tg, tglib::Distance_Type distanceType) {
    std::vector<double> closeness(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, closeness, distanceType)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        closeness[nid] = temporal_closeness(tg, nid, tg.getTimeInterval(), distanceType);
    }

    return closeness;
}

} // tglib

#endif //TGLIB_TEMPORALCLOSENESS_H
