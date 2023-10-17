/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalClusteringCoefficient.h
 *  @brief Contains function for computing the temporal clustering coefficient.
 */

#ifndef TGLIB_TEMPORALCLUSTERINGCOEFFICIENT_H
#define TGLIB_TEMPORALCLUSTERINGCOEFFICIENT_H

#include "../core/BasicTypes.h"
#include "../core/IncidentLists.h"

namespace tglib {

/**
 * @brief Computes the temporal clustering coefficient
 *
 * The temporal clustering coefficient is defined as
 * \f[
 * C_C(u) = \frac{\sum_{t\in T(\mathcal{G})} \pi_t(u)}{|T(\mathcal{G})|{|N(u)| \choose 2}},
 * \f]
 * where \f$\pi_t(u)=|\{(v,w,t,\lambda)\in\mathcal{E}\mid v,w\in N(u)\}|\f$
 * and \f$N(u)\f$ the neighbors of \f$u\f$ [1].
 *
 * [1] Tang, John, et al. "Temporal distance metrics for social network analysis."
 * Proceedings of the 2nd ACM workshop on Online social networks. 2009.
 *
 * @tparam N Node type
 * @param tg The temporal graph
 * @param nid The node for which the temporal clustering coefficient is computed
 * @param ti A restrictive time interval
 * @return The temporal clustering coefficient only considering edges in ti
 */
template<typename N, typename E>
double temporal_clustering_coefficient(IncidentLists<N, E> const& tg, NodeId nid, TimeInterval ti){

    std::set<NodeId> neighbors;
    std::set<Time> timesteps;
    for (auto &e : tg.getNode(nid).outEdges) {
        if (e.t < ti.first || e.t > ti.second) continue;
        neighbors.insert(e.v);
        timesteps.insert(e.t);
    }

    double count = 0;
    for (auto &v : neighbors) {
        for (auto &e : tg.getNode(v).outEdges) {
            if (neighbors.find(e.v) != neighbors.end()) {
                count += 1;
            }
        }
    }

    if (count == 0) return 0;

    auto m = (double)(neighbors.size() * (neighbors.size() - 1));
    double result = (1.0 / (double)timesteps.size()) * (count / m);

    return result;
}

}

#endif //TGLIB_TEMPORALCLUSTERINGCOEFFICIENT_H
