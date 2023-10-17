/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file TopologicalOverlap.h
 * @brief Contains function declarations for computing topological overlap.
 */

#ifndef TGLIB_TOPOLOGICALOVERLAP_H
#define TGLIB_TOPOLOGICALOVERLAP_H


#include <complex>
#include <algorithm>
#include "../core/BasicTypes.h"
#include "../core/IncidentLists.h"

namespace tglib {

/**
 * @brief Computes the topological overlap of a node.
 *
 * The topological overlap of a nodes is defined as
 * \f[
 * T_{to}(u)=\frac{1}{T(\mathcal{G})}\sum_{t=1}^{T(\mathcal{G})}\frac{\sum_{v\in\mathcal{N}(u)}\phi^t_{uv}\phi^{t+1}_{uv}}{\sqrt{\sum_{v\in\mathcal{N}(u)}\phi^t_{uv}\sum_{v\in\mathcal{N}(u)}\phi^{t+1}_{uv}}},
 * \f]
 * where \f$\phi^{t}_{uv}=1\f$ iff. there exists a temporal edges between \f$u\f$ and \f$v\f$ at time \f$t\f$ and zero otherwise [1].
 * In case that the denominator equals one, we define \f$T_{to}(u)=1\f$.
 * And, the global topological overlap is defined as
 * \f$T_{to}(\mathcal{G})=\frac{1}{N}\sum_{u\in V}T_{to}(u)\f$.
 * The topological overlap lies in the range between zero and one.
 * A value close to zero, means that many edges change and a value close to one means many edges change.
 *
 * [1]
 *
 * @tparam N The node type
 * @param tg The graph
 * @param nid The node id for which the topological overlap is computed
 * @param ti A restrictive time interval
 * @return The topological overlap for node nid in time interval ti
 */
template<typename N, typename E>
double topological_overlap(IncidentLists<N, E> const& tg, NodeId nid, TimeInterval ti) {
    std::vector<TemporalEdge> edges = tg.getNode(nid).outEdges;
    if (edges.empty()) return 0;

    std::sort(edges.begin(), edges.end());

    std::vector<std::set<NodeId>> counts;
    Time curTime = edges[0].t;
    counts.emplace_back();
    for (auto &e : edges) {
        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;
        if (e.t != curTime) {
            curTime = e.t;
            counts.emplace_back();
        }
        counts.back().insert(e.v);
    }

    double to = 0;
    for (size_t i = 0; i < counts.size()-1; ++i) {
        std::vector<int> v(std::min(counts[i].size(), counts[i+1].size()));
        auto it = std::set_intersection(counts[i].begin(), counts[i].end(),
                                        counts[i+1].begin(), counts[i+1].end(), v.begin());
        v.resize(it-v.begin());
        auto d = (double) v.size();
        auto n = (double) std::sqrt(counts[i].size() * counts[i+1].size());

        to += (d/n);
    }
    to /= ((double)counts.size() - 1.0);
    return to;
}


} // tglib


#endif //TGLIB_TOPOLOGICALOVERLAP_H
