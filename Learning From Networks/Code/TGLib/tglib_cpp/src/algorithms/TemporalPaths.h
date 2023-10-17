/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalPaths.h
 *  @brief Contains function declarations for computing temporal distances.
 */


#ifndef TGLIB_TEMPORALPATHS_H
#define TGLIB_TEMPORALPATHS_H

#include <memory>
#include <list>
#include <algorithm>
#include "../core/BasicTypes.h"
#include "../core/OrderedEdgeList.h"
#include "../core/IncidentLists.h"
#include "../util/LabelPQ.h"

namespace tglib {

/**
 * @brief Gets path from label sequence
 * @param path the resulting path
 * @param cur the last label at the end node of the path
 */
void fill_path(std::vector<tglib::TemporalEdge> &path, std::shared_ptr<PathLabel> cur) {
    while (cur->parent != nullptr) {
        path.push_back(cur->e);
        cur = cur->parent;
    }
    std::reverse(path.begin(), path.end());
}

/**
 * @brief Computes a fastest path between two nodes.
 *
 * @tparam N The node type
 * @param tg The temporal graph
 * @param start The start node
 * @param target The end node
 * @param ti The restrictive time interval
 * @return The path as a list of temporal edges
 */
template<typename N, typename E>
std::vector<tglib::TemporalEdge>
minimum_duration_path(tglib::IncidentLists<N, E> const &tg, tglib::NodeId start, tglib::NodeId target, tglib::TimeInterval ti) {
    std::vector<Time> durations(tg.getNumberOfNodes(), inf);
    durations[start] = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector<std::list<std::shared_ptr<PathLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<PathLabel>();
    l->a = l->s = l->d = 0;
    l->nid = start;
    nodelabels[start].push_back(l);

    PQ<PathLabel> q;
    q.push(l);

    size_t num_f = 0;

    std::shared_ptr<PathLabel> cur;
    bool found = false;

    while (!q.empty() && num_f < tg.getNumberOfNodes()) {

        cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        if (!visited[cur->nid]) {
            visited[cur->nid] = true;
            ++num_f;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<PathLabel>();;
                lnew->nid = e.v;
                if (cur->s == 0) lnew->s = e.t;
                else lnew->s = cur->s;
                lnew->a = e.t + e.tt;
                lnew->d = lnew->a - lnew->s;
                lnew->parent = cur;
                lnew->e = e;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    if (((*i)->s < lnew->s && (*i)->a >= lnew->a) || ((*i)->s == lnew->s && (*i)->a > lnew->a)) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->s >= lnew->s && (*i)->a <= lnew->a) || ((*i)->s == lnew->s && (*i)->a <= lnew->a)) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    durations[e.v] = durations[e.v] < lnew->d ? durations[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);
                }
            }
        }
        if (cur->nid == target) {
            found = true;
            break;
        }
    }

    std::vector<TemporalEdge> path;
    if (!found) return path;
    fill_path(path, cur);
    return path;
}

/**
 * @brief Computes an earliest arrival path between two nodes.
 *
 * @tparam N The node type
 * @param tg The temporal graph
 * @param start The start node
 * @param target The end node
 * @param ti The restrictive time interval
 * @return The path as a list of temporal edges
 */
template<typename N, typename E>
std::vector<tglib::TemporalEdge>
earliest_arrival_path(tglib::IncidentLists<N, E> const &tg, tglib::NodeId start, tglib::NodeId target, tglib::TimeInterval ti) {
    std::vector<Time> arrivalTimes(tg.getNumberOfNodes(), inf);
    arrivalTimes[start] = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector < std::list < std::shared_ptr < PathLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<PathLabel>();
    l->d = 0;
    l->nid = start;
    nodelabels.at(start).push_back(l);

    PQ<PathLabel> q;
    q.push(l);

    std::shared_ptr<PathLabel> cur;
    bool found = false;

    while (!q.empty()) {

        cur = q.top();
        q.pop();
        if (!visited[cur->nid]) {
            visited[cur->nid] = true;
        } else {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {
            if (e.t < ti.first || e.t + e.tt > ti.second) continue;
            if (e.t >= arrivalTimes[e.u]) {
                if (arrivalTimes[e.v] > e.t + e.tt) {
                    arrivalTimes[e.v] = e.t + e.tt;
                }
                auto lnew = std::make_shared<PathLabel>();;
                lnew->nid = e.v;
                lnew->d = e.t + e.tt;
                lnew->e = e;
                lnew->parent = cur;

                q.push(lnew);
            }
        }
        if (cur->nid == target) {
            found = true;
            break;
        }
    }

    std::vector<TemporalEdge> path;
    if (!found) return path;
    fill_path(path, cur);
    return path;
}

/**
 * @brief Computes a minimum transition time path between two nodes.
 *
 * @tparam N The node type
 * @param tg The temporal graph
 * @param start The start node
 * @param target The end node
 * @param ti The restrictive time interval
 * @return The path as a list of temporal edges
 */
template<typename N, typename E>
std::vector<tglib::TemporalEdge>
minimum_transition_time_path(tglib::IncidentLists<N, E> const &tg, tglib::NodeId start, tglib::NodeId target, tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[start] = 0;

    std::vector<std::list<std::shared_ptr<PathLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<PathLabel>();
    l->a = l->s = l->d = 0;
    l->nid = start;
    nodelabels[start].push_back(l);

    PQ<PathLabel> q;
    q.push(l);

    std::shared_ptr<PathLabel> cur;
    bool found = false;

    while (!q.empty() ) {

        cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<PathLabel>();;
                lnew->nid = e.v;
                lnew->a = e.t + e.tt;
                lnew->d = cur->d + e.tt;
                lnew->e = e;
                lnew->parent = cur;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    //    d' < d and a' <= a
                    // or d' = d and a' < a
                    if ((lnew->d < (*i)->d && lnew->a <= (*i)->a) /*|| (lnew->d == (*i)->d && lnew->a < (*i)->a)*/) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->d <= lnew->d && (*i)->a <= lnew->a) /*|| ((*i)->d == lnew->d && (*i)->a < lnew->a)*/) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    distances[e.v] = distances[e.v] < lnew->d ? distances[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);
                }
            }
        }
        if (cur->nid == target) {
            found = true;
            break;
        }
    }
    std::vector<TemporalEdge> path;
    if (!found) return path;
    fill_path(path, cur);
    return path;
}

/**
 * @brief Computes a minimum hops path between two nodes.
 *
 * @tparam N The node type
 * @param tg The temporal graph
 * @param start The start node
 * @param target The end node
 * @param ti The restrictive time interval
 * @return The path as a list of temporal edges
 */
template<typename N, typename E>
std::vector<tglib::TemporalEdge>
minimum_hops_path(tglib::IncidentLists<N, E> const &tg, tglib::NodeId start, tglib::NodeId target, tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[start] = 0;

    std::vector<std::list<std::shared_ptr<PathLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<PathLabel>();
    l->a = l->s = l->d = 0;
    l->nid = start;
    nodelabels[start].push_back(l);

    PQ<PathLabel> q;
    q.push(l);

    std::shared_ptr<PathLabel> cur;
    bool found = false;

    while (!q.empty() ) {

        cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<PathLabel>();;
                lnew->nid = e.v;
                lnew->a = e.t + e.tt;
                lnew->d = cur->d + 1;
                lnew->e = e;
                lnew->parent = cur;

                bool dom = false;

                auto i = nodelabels[e.v].begin();
                while (i != nodelabels[e.v].end()) {
                    //    d' < d and a' <= a
                    // or d' = d and a' < a
                    if ((lnew->d < (*i)->d && lnew->a <= (*i)->a) /*|| (lnew->d == (*i)->d && lnew->a < (*i)->a)*/) {
                        (*i)->deleted = true;
                        i = nodelabels[e.v].erase(i);
                        continue;
                    }
                    if (((*i)->d <= lnew->d && (*i)->a <= lnew->a) /*|| ((*i)->d == lnew->d && (*i)->a < lnew->a)*/) {
                        dom = true;
                        break;
                    }

                    ++i;
                }

                if (!dom) {
                    distances[e.v] = distances[e.v] < lnew->d ? distances[e.v] : lnew->d;
                    nodelabels[e.v].push_back(lnew);
                    q.push(lnew);
                }
            }
        }
        if (cur->nid == target) {
            found = true;
            break;
        }
    }
    std::vector<TemporalEdge> path;
    if (!found) return path;
    fill_path(path, cur);
    return path;
}

}


#endif //TGLIB_TEMPORALPATHS_H
