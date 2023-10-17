/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalKatz.h
 *  @brief Contains function declarations for computing temporal Katz centrality
 */

#ifndef TGLIB_TEMPORALKATZ_H
#define TGLIB_TEMPORALKATZ_H

#include <vector>
#include <map>
#include "../core/OrderedEdgeList.h"

namespace tglib {

/**
 * @brief Computes the temporal Katz centrality [1].
 *
 *
 *  [1] Béres, Ferenc, et al. "Temporal walk based centrality metric for graph streams."
 *  Applied network science 3.1 (2018): 1-26.
 *
 * @tparam E Temporal edge type
 * @param tgs Temporal edge stream
 * @param beta Parameter for weighting the walks
 * @return The temporal Katz centrality for each node
 */
template<typename E>
std::vector<double> temporal_katz_centrality(OrderedEdgeList<E> const &tgs, double beta) {

    std::vector<double> r(tgs.getNumberOfNodes(), 0);

    std::vector<std::map<Time, double>> walks(tgs.getNumberOfNodes(), std::map<Time, double>());
    for (auto &e : tgs.getEdges()) {
        walks.at(e.v)[e.t+1] += 1; //todo tt
        for (auto &x : walks.at(e.u)) {
            if (x.first > e.t) {
                continue;
            }
            walks.at(e.v)[e.t + 1] += (beta * x.second); //todo tt
        }
    }
    for (size_t i = 0; i < walks.size(); ++i) {
        for (auto &x : walks[i]) {
            r[i] += x.second;
        }
    }

    return r;
}

} // tglib

#endif //TGLIB_TEMPORALKATZ_H
