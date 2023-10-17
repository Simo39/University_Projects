/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TemporalReachability.h
 *  @brief Contains function declarations for computing temporal reachability.
 */


#ifndef TGLIB_REACHABILITY_H
#define TGLIB_REACHABILITY_H

#include "../core/BasicTypes.h"
#include "../core/OrderedEdgeList.h"
#include "../core/TRSGraph.h"
#include "../core/IncidentLists.h"
#include "../util/LabelPQ.h"

namespace tglib {

/**
 * @brief Computes the number of reachable nodes with a temporal walk starting at node id in the
 * time interval ti.
 *
 * Note that id reaches itself.
 *
 * @tparam E the edge type
 * @param tgs The temporal graph
 * @param nid The query node id
 * @param ti The restrictive time interval
 * @return The earliest arrival times
 */
template<typename E>
int64_t number_of_reachable_nodes(tglib::OrderedEdgeList<E> const &tgs, tglib::NodeId nid, tglib::TimeInterval ti) {
    std::vector<Time> arrival_time(tgs.getNumberOfNodes(), inf);
    arrival_time[nid] = 0;
    int64_t reachable = 1;

    for (auto &e : tgs.getEdges()) {

    if (e.t < ti.first) continue;
    if (e.t > ti.second) break;

    if (arrival_time[e.u] <= e.t && arrival_time[e.v] > e.t + e.tt) {
        if (arrival_time[e.v] == inf)
            reachable++;
        arrival_time[e.v] = e.t + e.tt;
    }
}

return reachable;
}

} // tglib


#endif //TGLIB_REACHABILITY_H
