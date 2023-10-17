/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalWalkCentrality.h
 *  @brief Contains function declarations for computing temporal walk centrality.
 */

#ifndef TGLIB_TEMPORALWALKCENTRALITY_H
#define TGLIB_TEMPORALWALKCENTRALITY_H


#include <map>
#include "../core/OrderedEdgeList.h"

namespace tglib {

/**
 * @brief counts incoming walks
 * @tparam E Temporal edge type
 * @param tgs The temporal graph
 * @param alpha weighting factor
 * @return walk counts
 */
template<typename E>
std::vector<std::map<Time, double>> getIncomingWeightedWalks(OrderedEdgeList<E> const &tgs, double alpha) {
    std::vector<std::map<Time, double>> walks(tgs.getNumberOfNodes(), std::map<Time, double>());
    for (auto &e: tgs.getEdges()) {
        walks.at(e.v)[e.t + 1] += 1; //alpha;
        for (auto &x: walks.at(e.u)) {
            if (x.first > e.t) {
                continue;
            }
            walks.at(e.v)[e.t + 1] += (alpha * x.second);
        }
    }
    return walks;
}

/**
 * @brief counts outgoing walks
 * @tparam E Temporal edge type
 * @param tgs The temporal graph
 * @param beta weighting factor
 * @return walk counts
 */
template<typename E>
std::vector<std::map<Time, double>> getOutgoingWeightedWalks(OrderedEdgeList<E> const &tgs, double beta) {
    std::vector<std::map<Time, double>> walks(tgs.getNumberOfNodes(), std::map<Time, double>());
    for (unsigned long ep = tgs.getEdges().size() - 1;; --ep) {
        auto &e = tgs.getEdges()[ep];
        walks.at(e.u)[e.t] += 1; //beta;
        for (auto &x: walks.at(e.v)) {
            if (x.first < e.t + 1) {
                continue;
            }
            walks.at(e.u)[e.t] += (beta * x.second);
        }
        if (ep == 0) break;
    }
    return walks;
}

/**
 * @brief Combines weighted temporal walk counts.
 *
 * @tparam E Temporal edge type
 * @param tgs The temporal graph
 * @param ifct incoming walk counts
 * @param ofct outgoing walk counts
 * @return the temporal walk centrality for all vertices
 */
template<typename E>
std::vector<double>
computeCentrality(OrderedEdgeList<E> const &tgs, std::vector<std::map<Time, double>> &ifct, std::vector<std::map<Time, double>> &ofct) {
    std::vector<double> centrality(tgs.getNumberOfNodes(), 0);

#pragma omp parallel for default(none) shared(centrality, tgs, ifct, ofct)
    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {

        auto init = ifct[nid].begin();
        auto outit = ofct[nid].begin();

        double insum = 0;

        while (outit != ofct[nid].end()) {
            if (init != ifct[nid].end() && outit != ofct[nid].end() && init->first < outit->first) {
                insum += init->second;
                ++init;
            }

            if (init != ifct[nid].end() && outit != ofct[nid].end() && outit->first < init->first) {
                if (init->first <= outit->first)
                    insum += init->second;
                centrality[nid] += insum * outit->second;
                ++outit;
            }
            if (init != ifct[nid].end() && outit != ofct[nid].end() && init->first == outit->first) {
                if (init->first <= outit->first)
                    insum += init->second;
                centrality[nid] += insum * outit->second;
                ++init;
                ++outit;
            }

            if (init == ifct[nid].end() && outit != ofct[nid].end()) {
                while (outit != ofct[nid].end()) {
                    centrality[nid] += insum * outit->second;
                    ++outit;
                }
                break;
            }
        }
    }
    return centrality;
}

/**
 * @brief Computes the temporal walk centrality [1].
 *
 * [1] Oettershagen, Lutz, Petra Mutzel, and Nils M. Kriege.
 * "Temporal Walk Centrality: Ranking Nodes in Evolving Networks."
 * Proceedings of the ACM Web Conference 2022. 2022.
 *
 * @tparam The temporal edge type
 * @param tgs the temporal graph
 * @param alpha the weighting factor for incoming walks
 * @param beta the weighting factor for outgoing walks
 * @return the centrality for each vertex
 */
template<typename E>
std::vector<double> temporal_walk_centrality(tglib::OrderedEdgeList<E> const &tgs,
                                                    double alpha, double beta) {
    std::vector<double> centrality;
    auto outwalks = getOutgoingWeightedWalks(tgs, beta);
    auto inwalks = getIncomingWeightedWalks(tgs, alpha);
    centrality = computeCentrality(tgs, inwalks, outwalks);
    return centrality;
}

} // tglib


#endif //TGLIB_TEMPORALWALKCENTRALITY_H
