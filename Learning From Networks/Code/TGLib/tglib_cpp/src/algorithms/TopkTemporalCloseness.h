/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TopkCloseness.h
 *  @brief Contains function declarations for computing top-k temporal closeness centrality
 */

#ifndef TGLIB_TOPKTEMPORALCLOSENESS_H
#define TGLIB_TOPKTEMPORALCLOSENESS_H

#include <algorithm>
#include "../core/IncidentLists.h"
#include "../util/TopkResult.h"
#include "../util/LabelPQ.h"

namespace tglib {

/**
 *
 * @tparam T
 * @param tg
 * @param nid
 * @param topkResult
 * @param ti
 * @param minTimeDelta
 */
template<typename T, typename E>
void calculate_closeness_with_bounding_duration(IncidentLists<T, E> &tg, NodeId nid, TopkResult<double> &topkResult,
                                                TimeInterval ti, double minTimeDelta) {

    std::vector <Time> minduration(tg.getNumberOfNodes(), inf);
    minduration.at(nid) = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector < std::list < std::shared_ptr < DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels.at(nid).push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    std::vector<bool> is_T_vertex(tg.getNumberOfNodes(), false);

    double total_reachable = tg.getNumberOfNodes();

    size_t num_f = 0;
    NodeId num_T = 0;
    double exact_closeness = 0.0;
    double cut_edges_approx = 0.0;

    std::vector<bool> exactAdded(tg.getNumberOfNodes(), false);


    while (!q.empty() && num_f < tg.getNumberOfNodes()) {

        auto cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        if (!visited.at(cur->nid)) {
            visited.at(cur->nid) = true;

            ++num_f;

            if (is_T_vertex[cur->nid]) {
                --num_T;
                cut_edges_approx -= 1.0 / cur->d;
            }

        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {
            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {
                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                if (cur->s == 0) lnew->s = e.t;
                else lnew->s = cur->s;
                lnew->a = e.t + e.tt;
                lnew->d = lnew->a - lnew->s;

                bool dom = false;

                auto i = nodelabels.at(e.v).begin();
                while (i != nodelabels.at(e.v).end()) {
                    if (((*i)->s < lnew->s && (*i)->a >= lnew->a) || ((*i)->s == lnew->s && (*i)->a > lnew->a)) {
                        (*i)->deleted = true;
                        i = nodelabels.at(e.v).erase(i);
                        continue;
                    }
                    if (((*i)->s >= lnew->s && (*i)->a <= lnew->a) || ((*i)->s == lnew->s && (*i)->a <= lnew->a)) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    minduration.at(e.v) = minduration.at(e.v) < lnew->d ? minduration.at(e.v) : lnew->d;
                    nodelabels.at(e.v).push_back(lnew);
                    q.push(lnew);

                    if (!is_T_vertex[e.v] && !visited[e.v]) {
                        ++num_T;
                        is_T_vertex[e.v] = true;
                    }

                }
            }
        }

        // update estimation for T vertices
        cut_edges_approx = (double) num_T / q.top()->d;

        // update exact value
        assert(minduration.at(cur->nid) < inf);
        if (minduration.at(cur->nid) > 0) {
            if (!exactAdded.at(cur->nid)) {
                exact_closeness += 1.0 / minduration.at(cur->nid);
                exactAdded.at(cur->nid) = true;
            }
        }

        // calculate approx
        if (topkResult.doApproximation()) {

            double rest_approx = double(total_reachable - num_f - num_T) / ((double) q.top()->d + 1.0 + minTimeDelta);
            double total_approx = exact_closeness + cut_edges_approx + rest_approx;

            if (total_approx < topkResult.getMinMaxTopk()) {
                return;
            }
        }
    }

    topkResult.insert(nid, exact_closeness);
}

template<typename T, typename E>
void calculate_closeness_with_bounding_shortest(tglib::IncidentLists<T, E> const &tg,
                                                tglib::NodeId nid, TopkResult<double> &topkResult,
                                                tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[nid] = 0;

    double total_reachable = tg.getNumberOfNodes();
    size_t num_f = 0;
    NodeId num_T = 0;
    std::vector<bool> exactAdded(tg.getNumberOfNodes(), false);
    double exact_closeness = 0.0;
    double cut_edges_approx = 0.0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    std::vector<bool> is_T_vertex(tg.getNumberOfNodes(), false);



    while (!q.empty() && num_f < tg.getNumberOfNodes()) {

        auto cur = q.top();
        q.pop();

        if (cur->deleted) {
            continue;
        }

        if (!visited.at(cur->nid)) {
            visited.at(cur->nid) = true;

            ++num_f;

            if (is_T_vertex[cur->nid]) {
                --num_T;
                cut_edges_approx -= 1.0 / cur->d;
            }

            if (distances[cur->nid] > 0 /*&& distances[cur->nid] < inf*/) {
                exact_closeness += 1.0 / (double)distances.at(cur->nid);
            }

        }


        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                lnew->a = e.t + e.tt;
                lnew->d = cur->d + e.tt;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    //    d' < d and a' <= a
                    // or d' = d and a' < a
                    if ((lnew->d < (*i)->d && lnew->a <= (*i)->a)) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->d <= lnew->d && (*i)->a <= lnew->a)) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    distances[e.v] = distances[e.v] < lnew->d ? distances[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);

                    if (!is_T_vertex[e.v] && !visited[e.v]) {
                        ++num_T;
                        is_T_vertex[e.v] = true;
                    }
                }

            }
        }

        // update estimation for T vertices
        cut_edges_approx = (double) num_T / q.top()->d;

        // calculate approx
        if (topkResult.doApproximation()) {

//            double rest_approx = double(total_reachable - num_f) / ((double) q.top()->d);
//            double total_approx = exact_closeness + rest_approx;
            double rest_approx = double(total_reachable - num_f - num_T) / ((double) q.top()->d + 1.0);
            double total_approx = exact_closeness + cut_edges_approx + rest_approx;

            if (total_approx < topkResult.getMinMaxTopk()) {
                return;
            }
        }
    }

    topkResult.insert(nid, exact_closeness);
}


/**
 * @brief Computes the top-k closeness values and nodes
 *
 * @param tg The temporal graph.
 * @param k Number of the topmost closeness values (and corresponding nodes) to compute
 * @param ti The restrictive time interval.
 * @param ti The distance type.
 * @return The top-k closeness values and nodes
 */
template<typename N, typename E>
std::vector<std::pair<tglib::NodeId, double>>
compute_topk_closeness(tglib::IncidentLists<N, E> &tg, int k,
                       tglib::TimeInterval ti,
                       tglib::Distance_Type distanceType) {
    std::vector<std::pair<NodeId, unsigned long>> perm;

    auto minTimeDelta = (double) inf;

    for (auto &n: tg.getNodes()) {
        perm.emplace_back(n.id, n.outEdges.size());

        if (n.outEdges.size() < 2) continue;
        for (size_t i = 0; i < n.outEdges.size() - 1; ++i) {
            if (std::abs(n.outEdges[i].t - n.outEdges[i + 1].t) < minTimeDelta)
                minTimeDelta = std::abs(n.outEdges[i].t - n.outEdges[i + 1].t);
        }
    }


    TopkResult<double> topkResult(k);

    std::sort(perm.begin(), perm.end(),
              [](const std::pair<NodeId, unsigned int> &a, const std::pair<NodeId, unsigned int> &b) -> bool {
                  return a.second > b.second;
              });

    for (auto p: perm) {
        switch (distanceType) {
            case Distance_Type::Earliest_Arrival:
//                calculate_closeness_with_bounding_ea(tg, p.first, topkResult, ti);
// todo
                break;
            case Distance_Type::Fastest:
                calculate_closeness_with_bounding_duration(tg, p.first, topkResult, ti, minTimeDelta);
                break;
            case Distance_Type::Latest_Departure:
                // todo
                break;
            case Distance_Type::Minimum_Transition_Times:
                calculate_closeness_with_bounding_shortest(tg, p.first, topkResult, ti);
                break;
            case Distance_Type::Minimum_Hops:
                // todo
                break;
        }
    }

    return topkResult.getResults();
}


} // tglib

#endif //TGLIB_TOPKTEMPORALCLOSENESS_H
