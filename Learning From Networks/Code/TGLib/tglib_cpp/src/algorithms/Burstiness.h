/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file Burstiness.h
 *  @brief Contains function declarations for computing edge_burstiness
 */

#ifndef TGLIB_BURSTINESS_H
#define TGLIB_BURSTINESS_H


#include "../core/OrderedEdgeList.h"
#include "../util/UtilFunctions.h"
#include <map>

namespace tglib {

/**
 * @brief Computes the edge burstiness for all (static) edges
 *
 * Burstiness measures how much a sequence of contacts \f$\tau\f$
 * of a pair of edges deviates from the
 * memoryless random Poisson process [1].
 * It is defined as
 * \f[
 *      B(\tau)=\frac{\sigma_\tau - m_\tau}{\sigma_\tau + m_\tau} \in [-1,1],
 * \f]
 * where \f$\sigma_\tau\f$ and \f$m_\tau\f$ denote the standard deviation and mean
 * of the inter-contact times \f$\tau\f$, respectively [1].
 * A value close to one indicates a very bursty sequence, and a value
 * close to minus one a more periodic sequence.
 *
 * [1] Goh, K-I., and A-L. Barabási. "Burstiness and memory in complex systems."
 * EPL (Europhysics Letters) 81.4 (2008): 48002.
 *
 * @tparam E The temporal edge type
 * @param tgs The temporal graph as ordered edge list
 * @param ti The time interval for which the edge_burstiness is computed
 * @return For each pair of nodes with at least one temporal edge connecting them, the edge_burstiness of the tie
 */
template<typename E>
std::map<std::pair<tglib::NodeId, tglib::NodeId>, double>
edge_burstiness(tglib::OrderedEdgeList<E> const &tgs,
                tglib::TimeInterval ti) {
    std::map<std::pair<NodeId, NodeId>, double> result;
    std::map<std::pair<NodeId, NodeId>, std::vector<Time>> inter_contact_times;
    std::vector<Time> last_time(tgs.getEdges().size());

    for (auto &e : tgs.getEdges()) {
        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;
        inter_contact_times[{e.u, e.v}].push_back(e.t);
    }


#pragma omp parallel default(none) shared(inter_contact_times, result)
    for (auto &p : inter_contact_times) {
        if (p.second.size() < 2) continue;
        Time lt = p.second[0];
        std::vector<Time> ict(p.second.size()-1);
        for (size_t i = 1; i < p.second.size(); ++i) {
            Time nt = p.second[i];
            ict[i - 1] = nt - lt;
            lt = nt;
        }

        double m = 0, s = 0;
        get_mean_std(ict, m, s);
#pragma omp critical
        result[p.first] = (s - m) / (s + m);
    }

    return result;
}

/**
 * @brief Computes the edge burstiness for all nodes
 *
 * Burstiness measures how much a sequence of contacts \f$\tau\f$
 * of a single node  deviates from the
 * memoryless random Poisson process [1].
 * It is defined as
 * \f[
 *      B(\tau)=\frac{\sigma_\tau - m_\tau}{\sigma_\tau + m_\tau} \in [-1,1],
 * \f]
 * where \f$\sigma_\tau\f$ and \f$m_\tau\f$ denote the standard deviation and mean
 * of the inter-contact times \f$\tau\f$, respectively [1].
 * A value close to one indicates a very bursty sequence, and a value
 * close to minus one a more periodic sequence.
 *
 * [1] Goh, K-I., and A-L. Barabási. "Burstiness and memory in complex systems."
 * EPL (Europhysics Letters) 81.4 (2008): 48002.
 *
 * @tparam E The temporal edge type
 * @param tgs The temporal graph as ordered edge list
 * @param ti The time interval for which the node burstiness is computed
 * @return For each node the node burstiness
 */
template<typename E>
std::vector<double>
node_burstiness(tglib::OrderedEdgeList<E> const &tgs,
                tglib::TimeInterval ti){
    std::vector<double> result(tgs.getNumberOfNodes(), 0);
//    std::vector<double> result(tgs.getNumberOfNodes(), std::numeric_limits<double>::max());
    std::vector<std::vector<Time>> inter_contact_times(tgs.getNumberOfNodes(), std::vector<Time>());
    std::vector<Time> last_time(tgs.getEdges().size());

    for (auto &e : tgs.getEdges()) {
        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;
        inter_contact_times[e.u].push_back(e.t);
    }

#pragma omp parallel default(none) shared(inter_contact_times, tgs, result)
    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto &p = inter_contact_times[nid];
        if (p.size() < 2) continue;
        std::vector<Time> ict(p.size()-1);
        Time lt = p[0];
        for (size_t i = 1; i < p.size(); ++i) {
            Time nt = p[i];
            ict[i - 1] = nt - lt;
            lt = nt;
        }

        double m = 0, s = 0;
        get_mean_std(ict, m, s);
#pragma omp critical
        result[nid] = (s - m) / (s + m);
    }

    return result;
}

} // tglib

#endif //TGLIB_BURSTINESS_H
