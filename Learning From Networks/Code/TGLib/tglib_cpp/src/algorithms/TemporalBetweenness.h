/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalBetweenness.h
 *  @brief Contains function for computing temporal edge betweenness
 */

#ifndef TGLIB_TEMPORALBETWEENNESS_H
#define TGLIB_TEMPORALBETWEENNESS_H

#include <stack>
#include <queue>
#include <map>
#include "../core/DirectedLineGraph.h"
#include "../core/OrderedEdgeList.h"
#include "../core/Transformations.h"

namespace tglib {

/**
 * @brief Computes the temporal edge betweenness
 *
 * The temporal edge betweenness is defined as the number of min hop paths
 * passing over an edge.
 *
 * @param dlg Directed line graph representation of the temporal graph
 * @return The temporal edge betweenness of each edge
 */
inline std::vector<double> temporal_edge_betweenness(tglib::DirectedLineGraph const &dlg) {

    std::vector<double> betweenness(dlg.num_tg_nodes, 0);
    std::vector<double> betweenness_dlg(dlg.nodes.size(), 0);

    // For every node being starting node
    for (size_t s = 0; s < dlg.nodes.size(); s++) {
        std::stack<NodeId> S;

        std::vector<Distance> d(dlg.nodes.size(), -1);
        d[s] = 0;

        std::vector<NodeId> prev(dlg.nodes.size(), 0);
        std::queue<NodeId> Q;
        Q.push(s);

        std::vector<double> sigma(dlg.nodes.size(), 0);
        sigma[s] = 1;

        std::map<NodeId, std::vector<NodeId>> P; // neighbours visited

        // Brandes' alg.
        while(!Q.empty()) {
            int u = Q.front();
            Q.pop();
            S.push(u);

            for (auto &e : dlg.nodes[u].outedges) {
                NodeId v = e.v;
                if(d[v] < 0) {
                    Q.push(v);
                    d[v] = d[u] + 1;
                }

                // Update sigma if shortest path
                if(d[v] == d[u] + 1) {
                    sigma[v] = sigma[v] + sigma[u];
                    P[v].push_back(u);
                }
            }
        }
        std::vector<double> delta(dlg.nodes.size(), 0);

        while (!S.empty()) {
            int w = S.top();
            S.pop();
            for(auto it = P[w].begin(); it != P[w].end(); ++it)    {
                int v = *it;
                delta[v] += ((sigma[v]/sigma[w])*(1+ delta[w]));
            }
            if ((size_t)w != s) {
                betweenness_dlg[w] += delta[w];
            }
        }
    }

    return betweenness_dlg;
}

/**
 * @brief Computes the temporal edge betweenness
 *
 * The temporal edge betweenness is defined as the number of min hop paths
 * passing over an edge.
 * @tparam E The temporal edge type
 * @param tgs The temporal graph
 * @return The temporal edge betweenness of each edge
 */
template<typename E>
std::vector<double> temporal_edge_betweenness(tglib::OrderedEdgeList<E> const &tgs) {
    auto tg = to_incident_lists<TGNode>(tgs);
    auto dlg = to_directed_line_graph(tg);
    return temporal_edge_betweenness(dlg);
}

} // tglib

#endif //TGLIB_TEMPORALBETWEENNESS_H
