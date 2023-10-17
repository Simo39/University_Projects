/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalDistances.h
 *  @brief Contains function declarations for computing temporal distances.
 */

#ifndef TGLIB_TEMPORALDISTANCES_H
#define TGLIB_TEMPORALDISTANCES_H


#include <vector>
#include <list>
#include <memory>
#include "../core/BasicTypes.h"
#include "../core/OrderedEdgeList.h"
#include "../core/TRSGraph.h"
#include "../core/IncidentLists.h"
#include "../util/LabelPQ.h"

namespace tglib {

/**
 * @brief Computes the durations of fastest paths from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The durations of the fastest paths
 */
template<typename E>
std::vector<tglib::Time>
minimum_durations(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> durs(tgs.getNumberOfNodes(), inf);
    durs[nid] = 0;
    std::vector<std::set<std::pair<Time, Time>>> node_labels(tgs.getNumberOfNodes(), std::set<std::pair<Time, Time>>());

    for (auto &e: tgs.getEdges()) {
        if (e.t >= ti.second) break;
        if (e.t + e.tt > ti.second || e.t < ti.first) continue;

        if (e.u == nid) {
            node_labels[e.u].insert(std::make_pair(e.t, e.t));
        }

        if (node_labels[e.u].empty()) continue;

        auto nodeLabelIt = node_labels[e.u].upper_bound(std::make_pair(e.t, inf));

        if (nodeLabelIt != node_labels[e.u].begin()) {
            nodeLabelIt--;

            Time arrivalTime = e.t + e.tt;
            Time startTime = nodeLabelIt->second;

            if (arrivalTime - startTime < durs[e.v]) {
                durs[e.v] = arrivalTime - startTime;
            }

            if (node_labels[e.v].empty()) {
                node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                continue;
            }

            auto lowerBound = node_labels[e.v].lower_bound(std::make_pair(arrivalTime, startTime));

            if (lowerBound == node_labels[e.v].begin()) {
                if (lowerBound->first > arrivalTime) {
                    auto upperBound = lowerBound;
                    for (; upperBound != node_labels[e.v].end(); upperBound++) {
                        if (upperBound->second > startTime) {
                            break;
                        }
                    }

                    node_labels[e.v].erase(lowerBound, upperBound);
                    node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                }
            } else if (lowerBound == node_labels[e.v].end()) {
                lowerBound--;
                if (lowerBound->first == arrivalTime) {
                    node_labels[e.v].erase(lowerBound);
                    node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                } else if (lowerBound->second < startTime) {
                    node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                }
            } else {
                if (lowerBound->first > arrivalTime) {
                    auto oneBeforeLowerBound = lowerBound;
                    oneBeforeLowerBound--;

                    if (oneBeforeLowerBound->first == arrivalTime) {
                        for (; lowerBound != node_labels[e.v].end(); lowerBound++) {
                            if (lowerBound->second > startTime) {
                                break;
                            }
                        }

                        node_labels[e.v].erase(oneBeforeLowerBound, lowerBound);
                        node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                    } else if (oneBeforeLowerBound->second < startTime) {
                        auto upperBound = lowerBound;
                        for (; upperBound != node_labels[e.v].end(); upperBound++) {
                            if (upperBound->second > startTime) {
                                break;
                            }
                        }

                        node_labels[e.v].erase(lowerBound, upperBound);
                        node_labels[e.v].insert(std::make_pair(arrivalTime, startTime));
                    }
                }
            }

        }
    }

    return durs;
}

/**
 * @brief Computes the earliest arrival times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @tparam E the edge type
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
template<typename E>
std::vector<tglib::Time>
earliest_arrival_times(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrival_time(tgs.getNumberOfNodes(), inf);
    arrival_time[nid] = 0;

    for (auto &e : tgs.getEdges()) {

        if (e.t < ti.first) continue;
        if (e.t > ti.second) break;

        if (arrival_time[e.u] <= e.t && arrival_time[e.v] > e.t + e.tt) {
            arrival_time[e.v] = e.t + e.tt;
        }
    }

    return arrival_time;
}


/**
 * @brief Computes the latest departure times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The lastest departure times
 */
template<typename E>
std::vector<tglib::Time>
latest_departure_times(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrival_time(tgs.getNumberOfNodes(), -1);
    arrival_time[nid] = 0;

    for (long i = (long)tgs.getEdges().size()-1; i >= 0; --i) {
        auto &e = tgs.getEdges()[i];

        if (e.t < ti.first) break;

        if (arrival_time[e.u] >= e.t + e.tt && arrival_time[e.v] < e.t) {
            arrival_time[e.v] = e.t;
        }
    }

    return arrival_time;
}

/**
 * @brief Computes the minimum number of hops from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum number of hops
 */
template<typename E>
std::vector<tglib::Distance>
minimum_hops(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> distances(tgs.getNumberOfNodes(), inf);

    distances[nid] = 0;
    std::vector<std::set<std::pair<Time, Distance> > > nodeLabels;
    nodeLabels.resize(tgs.getNumberOfNodes());

    for (auto &e: tgs.getEdges()) {
        if (e.t >= ti.second) break;
        if (e.t + e.tt > ti.second || e.t < ti.first) continue;

        if (e.u == nid) {
            nodeLabels[e.u].insert(std::make_pair(e.t, 0));
        }

        if (nodeLabels[e.u].empty()) continue;

        auto upperBound = nodeLabels[e.u].upper_bound({e.t, inf});

        if (upperBound != nodeLabels[e.u].begin()) {
            upperBound--;

            auto arrivalTime = e.t + e.tt;
            auto distance = upperBound->second + 1;

            if (distance < distances[e.v])
                distances[e.v] = distance;

            if (!nodeLabels[e.v].empty()) {
                auto bound = nodeLabels[e.v].upper_bound({arrivalTime, distance});

                if (bound == nodeLabels[e.v].begin()) {
                    auto boundLow = bound, bouldUp = bound;
                    for (; bouldUp != nodeLabels[e.v].end(); bouldUp++) {
                        if (bouldUp->second < distance) {
                            break;
                        }
                    }

                    nodeLabels[e.v].erase(boundLow, bouldUp);
                    nodeLabels[e.v].insert({arrivalTime, distance});

                } else if (bound == nodeLabels[e.v].end()) {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                } else {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        auto boundLow = bound, boundUp = bound;
                        for (; boundUp != nodeLabels[e.v].end(); boundUp++) {
                            if (boundUp->second < distance) {
                                break;
                            }
                        }

                        nodeLabels[e.v].erase(boundLow, boundUp);
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                }
            } else {
                nodeLabels[e.v].insert({arrivalTime, distance});
            }
        }
    }
    return distances;
}

/**
 * @brief Computes the minimum sum of transition times from the node with id nid to all, other during the
 * time interval ti reachable, nodes.
 *
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum sum of transition times
 */
template<typename E>
std::vector<tglib::Time>
minimum_transition_times(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid,
                         tglib::TimeInterval ti) {
    std::vector<Time> distances(tgs.getNumberOfNodes(), inf);

    distances[nid] = 0;
    std::vector<std::set<std::pair<Time, Distance> > > nodeLabels;
    nodeLabels.resize(tgs.getNumberOfNodes());

    for (auto &e: tgs.getEdges()) {
        if (e.t >= ti.second) break;
        if (e.t + e.tt > ti.second || e.t < ti.first) continue;

        if (e.u == nid) {
            nodeLabels[e.u].insert(std::make_pair(e.t, 0));
        }

        if (nodeLabels[e.u].empty()) continue;

        auto upperbound = nodeLabels[e.u].upper_bound({e.t, inf});

        if (upperbound != nodeLabels[e.u].begin()) {
            upperbound--;

            auto arrivalTime = e.t + e.tt;
            auto distance = upperbound->second + e.tt;

            if (distance < distances[e.v])
                distances[e.v] = distance;

            if (!nodeLabels[e.v].empty()) {
                auto bound = nodeLabels[e.v].upper_bound({arrivalTime, distance});

                if (bound == nodeLabels[e.v].begin()) {
                    auto boundLow = bound, boundUp = bound;
                    for (; boundUp != nodeLabels[e.v].end(); boundUp++) {
                        if (boundUp->second < distance) {
                            break;
                        }
                    }

                    nodeLabels[e.v].erase(boundLow, boundUp);
                    nodeLabels[e.v].insert({arrivalTime, distance});

                } else if (bound == nodeLabels[e.v].end()) {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                } else {
                    auto boundOneBefore = bound;
                    boundOneBefore--;

                    if (boundOneBefore->first < arrivalTime && boundOneBefore->second > distance) {
                        auto boundLow = bound, boundUp = bound;
                        for (; boundUp != nodeLabels[e.v].end(); boundUp++) {
                            if (boundUp->second < distance) {
                                break;
                            }
                        }

                        nodeLabels[e.v].erase(boundLow, boundUp);
                        nodeLabels[e.v].insert({arrivalTime, distance});
                    }
                }
            } else {
                nodeLabels[e.v].insert({arrivalTime, distance});
            }
        }
    }

    return distances;
}



/**
 * @brief Computes the durations of fastest paths from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @tparam The node type
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The durations of the fastest paths
 */
template<typename N, typename E>
std::vector<tglib::Time>
minimum_durations(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> durations(tg.getNumberOfNodes(), inf);
    durations[nid] = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    size_t num_f = 0;

    while (!q.empty() && num_f < tg.getNumberOfNodes()) {

        auto cur = q.top();
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

                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                if (cur->s == 0) lnew->s = e.t;
                else lnew->s = cur->s;
                lnew->a = e.t + e.tt;
                lnew->d = lnew->a - lnew->s;

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
    }
    return durations;
}

/**
 * @brief Computes the earliest arrival times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @tparam E the node type
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
template<typename N, typename E>
std::vector<tglib::Time>
earliest_arrival_times(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrivalTimes(tg.getNumberOfNodes(), inf);
    arrivalTimes[nid] = 0;

    std::vector<bool> visited(tg.getNumberOfNodes(), false);

    std::vector < std::list < std::shared_ptr < DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->d = 0;
    l->nid = nid;
    nodelabels.at(nid).push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    while (!q.empty()) {

        auto cur = q.top();
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
                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                lnew->d = e.t + e.tt;

                q.push(lnew);
            }
        }
    }

    return arrivalTimes;
}


/**
 * @brief Computes the minimum sum of transition times from the node with id nid to all, other during the
 * time interval ti reachable, nodes.
 *
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum sum of transition times
 */
template<typename N, typename E>
std::vector<tglib::Distance>
minimum_transition_times(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[nid] = 0;

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    while (!q.empty() ) {

        auto cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
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
    }
    return distances;
}

/**
 * @brief Computes the minimum number of hops from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param tg The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum number of hops
 */
template<typename N, typename E>
std::vector<tglib::Distance>
minimum_hops(tglib::IncidentLists<N, E> const &tg, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> distances(tg.getNumberOfNodes(), inf);
    distances[nid] = 0;

    std::vector<std::list<std::shared_ptr<DistanceLabel>>> nodelabels(tg.getNumberOfNodes());
    auto l = std::make_shared<DistanceLabel>();
    l->a = l->s = l->d = 0;
    l->nid = nid;
    nodelabels[nid].push_back(l);

    PQ<DistanceLabel> q;
    q.push(l);

    while (!q.empty() ) {

        auto cur = q.top();
        q.pop();
        if (cur->deleted) {
            continue;
        }

        for (auto &e: tg.getNode(cur->nid).outEdges) {

            if (e.t < ti.first || e.t + e.tt > ti.second) continue;

            if (e.t >= cur->a) {

                auto lnew = std::make_shared<DistanceLabel>();;
                lnew->nid = e.v;
                lnew->a = e.t + e.tt;
                lnew->d = cur->d + 1;

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
    }
    return distances;
}


/**
 * @brief Computes the durations of fastest paths from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph in TRSGraph representation
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The durations of the fastest paths
 */
inline std::vector<tglib::Time>
minimum_durations(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<bool> visited(trs.nodes.size(), false);
    std::vector<Time> durs(trs.numTGNodes, inf);
    durs[nid] = 0;
    std::vector<NodeId> stack;
    stack.reserve(trs.nodes.size());

    auto startPos = trs.positionOfLatestTimeNode[nid];

    // the nodes in trs are ordered sequential for a tg-node in increasing time
    while (startPos >=0 && trs.nodes[startPos].tn.first == nid) {
        auto &nextNode = trs.nodes[startPos];
        startPos--;

        if (nextNode.tn.second < ti.first) break;    // all next nodes would be not later
        if (nextNode.tn.second > ti.second) continue;

        visited[nextNode.nid] = true;
        stack.push_back(nextNode.nid);

        Time startTime = nextNode.tn.second;

        while (!stack.empty()) {
            auto &curNode = trs.nodes[stack.back()];
            stack.pop_back();

            for (auto &e : curNode.adjList) {
                Time arrTime = curNode.tn.second + e.tt;
                Time duration = arrTime - startTime;
                NodeId v = trs.nodes[e.v].tn.first;
                if (arrTime <= ti.second && duration < durs[v]) {
                    durs[v] = duration;
                }

                if (!visited[e.v]) {
                    visited[e.v] = true;
                    if (trs.nodes[e.v].tn.second <= ti.second) {
                        stack.push_back(e.v);
                        size_t s = e.v;
                        while (s < trs.nodes.size() && trs.nodes[s].tn.first == e.v) {
                            if (trs.nodes[s].tn.second > ti.second) break;
                            if (!visited[trs.nodes[s].nid]) {
                                visited[trs.nodes[s].nid] = true;
                                stack.push_back(trs.nodes[s].nid);
                            }
                            s++;
                        }
                    }
                }
            }
        }
    }

    return durs;
}


/**
 * @brief Computes the earliest arrival times from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph in TRSGraph representation
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
inline std::vector<tglib::Time>
earliest_arrival_times(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<bool> visited(trs.nodes.size(), false);
    std::vector<Time> arrival_times(trs.numTGNodes, inf);
    arrival_times[nid] = 0;
    std::vector<NodeId> stack;
    stack.reserve(trs.nodes.size());

    auto startPos = trs.positionOfEarliestTimeNode[nid];

    // the nodes in trs are ordered sequential for a tg-node in increasing time
    while (startPos <= trs.positionOfLatestTimeNode[nid]) {
        auto &nextNode = trs.nodes[startPos];
        startPos++;

        if (nextNode.tn.second < ti.first) continue;
        if (nextNode.tn.second > ti.second) break;

        visited[nextNode.nid] = true;
        stack.push_back(nextNode.nid);

        while (!stack.empty()) {
            auto &curNode = trs.nodes[stack.back()];
            stack.pop_back();

            for (auto &e : curNode.adjList) {
                if (visited[e.v]) continue;
                visited[e.v] = true;
                stack.push_back(e.v);

                Time arrTime = curNode.tn.second + e.tt;
                NodeId v = trs.nodes[e.v].tn.first;
                if (arrTime < ti.second && arrTime < arrival_times[v]) {
                    arrival_times[v] = arrTime;
                }
            }

        }
    }

    return arrival_times;
}


/**
 * @brief Computes the minimum number of hops from the node with id nid to all other, during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum number of hops
 */
inline std::vector<tglib::Distance>
minimum_hops(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti){
    std::vector<Distance> distances(trs.nodes.size(), inf);

    // todo corner case: vertex is isolated
    if (trs.positionOfEarliestTimeNode[nid] == inf) {
        std::vector<Distance> result(trs.numTGNodes, inf);
        result[nid] = 0;
        return result;
    }

    distances[trs.positionOfEarliestTimeNode[nid]] = 0;

    std::vector<bool> visited(trs.nodes.size(), false);

    auto cmp=[](const std::pair<Distance, NodeId>& a,const std::pair<Distance, NodeId>& b){
        return a.first > b.first;
    };
    std::priority_queue<std::pair<Distance, NodeId>,std::vector<std::pair<Distance, NodeId>>,decltype(cmp)> q(cmp);

    q.push({0, trs.positionOfEarliestTimeNode[nid]});

    while (!q.empty()) {

        auto cur = q.top();
        q.pop();

        auto &u = trs.nodes[cur.second];
        Time u_t = u.tn.second;

        if (visited[u.nid] || u_t > ti.second || u_t < ti.first) {
            continue;
        }
        visited[u.nid] = true;

        for (auto &e : u.adjList) {
            auto &v = trs.nodes[e.v];
            if (v.tn.first == u.tn.first) {
                if (distances[v.nid] > cur.first)
                    distances[v.nid] = cur.first;
                q.push({distances[v.nid], v.nid});
                continue;
            }
            if (distances[v.nid] > cur.first + 1)
                distances[v.nid] = distances[u.nid] + 1;
            q.push({distances[v.nid], v.nid});
        }
    }

    std::vector<Distance> result(trs.numTGNodes, inf);
    for (size_t i = 0; i < trs.nodes.size(); ++i) {
        result[trs.nodes[i].tn.first] = std::min(distances[i], result[trs.nodes[i].tn.first]);
    }
    return result;
}


/**
 * @brief Computes the minimum sum of transition times from the node with id nid to all, other during the
 * time interval ti reachable, nodes.
 *
 * @param trs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The minimum sum of transition times
 */
inline std::vector<tglib::Distance>
minimum_transition_times(tglib::TRSGraph const &trs, tglib::NodeId nid, tglib::TimeInterval ti){
    std::vector<Distance> distances(trs.nodes.size(), inf);

    // todo corner case: vertex is isolated
    if (trs.positionOfEarliestTimeNode[nid] == inf) {
        std::vector<Distance> result(trs.numTGNodes, inf);
        result[nid] = 0;
        return result;
    }

    distances[trs.positionOfEarliestTimeNode[nid]] = 0;

    std::vector<bool> visited(trs.nodes.size(), false);

    auto cmp=[](const std::pair<Distance, NodeId>& a,const std::pair<Distance, NodeId>& b){
        return a.first > b.first;
    };
    std::priority_queue<std::pair<Distance, NodeId>,std::vector<std::pair<Distance, NodeId>>,decltype(cmp)> q(cmp);

    q.push({0, trs.positionOfEarliestTimeNode[nid]});

    while (!q.empty()) {

        auto cur = q.top();
        q.pop();

        auto &u = trs.nodes[cur.second];
        Time u_t = u.tn.second;

        if (visited[u.nid] || u_t > ti.second || u_t < ti.first) {
            continue;
        }
        visited[u.nid] = true;

        for (auto &e : u.adjList) {
            auto &v = trs.nodes[e.v];
            if (v.tn.first == u.tn.first) {
                if (distances[v.nid] > cur.first)
                    distances[v.nid] = cur.first;
                q.push({distances[v.nid], v.nid});
                continue;
            }
            if (distances[v.nid] > cur.first + e.tt)
                distances[v.nid] = distances[u.nid] + e.tt;
            q.push({distances[v.nid], v.nid});
        }
    }

    std::vector<Distance> result(trs.numTGNodes, inf);
    for (size_t i = 0; i < trs.nodes.size(); ++i) {
        result[trs.nodes[i].tn.first] = std::min(distances[i], result[trs.nodes[i].tn.first]);
    }
    return result;
}


} // tglib


#endif //TGLIB_TEMPORALDISTANCES_H
