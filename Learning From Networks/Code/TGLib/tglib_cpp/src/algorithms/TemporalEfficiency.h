/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalEfficiency.h
 *  @brief Contains function declarations for computing temporal efficiency.
 */

#ifndef TGLIB_TEMPORALEFFICIENCY_H
#define TGLIB_TEMPORALEFFICIENCY_H

#include <vector>
#include "../core/BasicTypes.h"
#include "../algorithms/TemporalCloseness.h"

/**
 * @brief Computes the temporal efficiency [1] for the temporal graph.
 *
 * The temporal efficiency is a global statistic based on the temporal closeness values of the nodes [1].
 * It is defined as
 * \f[
 * T_{eff}(\mathcal{G})=\frac{1}{n(n-1)}\sum_{u\in V(\mathcal{G})}\sum_{v\neq u\in V(\mathcal{G})}\frac{1}{d(u,v)}
 * \f]
 * with \f$d(u,v)\f$ being a temporal distance and \f$1/\infty=0\f$ in case of non-reachable vertices.
 *
 * [1]Tang, John, et al. "Temporal distance metrics for social network analysis."
 * Proceedings of the 2nd ACM workshop on Online social networks. 2009.
 *
 * @tparam TempGraph Temporal graph type.
 * @param tg the temporal graph
 * @param timeInterval the restrictive time interval
 * @param distanceType the temporal distance type
 * @return the efficiency of tg
 */
template<typename TempGraph>
double temporal_efficiency(TempGraph const& tg, tglib::TimeInterval timeInterval, tglib::Distance_Type distanceType) {
    std::vector<double> closeness(tg.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(tg, closeness, distanceType, timeInterval)
    for (size_t nid = 0; nid < tg.getNumberOfNodes(); ++nid) {
        closeness[nid] = temporal_closeness(tg, nid, timeInterval, distanceType);
    }

    double efficiency = 0;
    for (auto &c : closeness) {
        efficiency += c;
    }
    return 1.0/(tg.getNumberOfNodes()*(tg.getNumberOfNodes()-1)) * efficiency;
}

#endif //TGLIB_TEMPORALEFFICIENCY_H
