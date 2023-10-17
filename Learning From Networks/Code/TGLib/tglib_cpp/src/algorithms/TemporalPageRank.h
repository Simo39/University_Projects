/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalPageRank.h
 *  @brief Contains function declarations for computing TemporalPageRank centrality
 */

#ifndef TGLIB_TEMPORALPAGERANK_H
#define TGLIB_TEMPORALPAGERANK_H

#include <vector>
#include "../core/OrderedEdgeList.h"


namespace tglib {
/**
 * @brief Computes the temporal PageRank [1]
 *
 *
 * [1] Rozenshtein, Polina, and Aristides Gionis. "Temporal pagerank."
 * Joint European Conference on Machine Learning and Knowledge Discovery in Databases.
 * Springer, Cham, 2016.
 *
 * @tparam E The temporal edge type
 * @param tgs The ordered edge stream with edge type E
 * @param alpha Weighting factor
 * @param beta Weighting factor
 * @param gamma Weighting factor
 * @return The temporal PageRank centrality for each node
 */
template<typename E>
std::vector<double> temporal_pagerank(OrderedEdgeList<E> const &tgs, double alpha, double beta, double gamma) {

    std::vector<double> degrees(tgs.getNumberOfNodes(), 0.0);
    for (auto &e : tgs.getEdges()) {
        degrees.at(e.u)++;
    }
    for (auto &d : degrees) d /= tgs.getEdges().size();

    double normalization = 1.0;
    if (beta == 1.0) beta = 0.0;

    std::vector<double> r(tgs.getNumberOfNodes(), 0.0);
    std::vector<double> s(tgs.getNumberOfNodes(), 0.0);

    for (auto &e : tgs.getEdges()) {
        r[e.u] = r[e.u] * gamma + 1.0 * (1.0 - alpha) * degrees[e.u] * alpha * normalization;
        r[e.v] = r[e.v] * gamma + s[e.u] + 1.0 * (1.0 - alpha) * degrees[e.u] * alpha * normalization;
        s[e.v] = s[e.v] + (s[e.u] + 1.0 * (1.0 - alpha)* s[e.u]) * alpha * (1.0 - beta);
        s[e.u] = s[e.u] * beta;
    }

    return r;
}

} // tglib


#endif //TGLIB_TEMPORALPAGERANK_H
