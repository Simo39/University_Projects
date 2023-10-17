/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file Transformations.h
 *  @brief Contains functions for transforming temporal graph between different representations
 */

#ifndef TGLIB_TRANSFORMATIONS_H
#define TGLIB_TRANSFORMATIONS_H

#include <map>
#include <algorithm>
#include "OrderedEdgeList.h"
#include "IncidentLists.h"
#include "TRSGraph.h"
#include "DirectedLineGraph.h"
#include "AggregatedGraph.h"
#include <cassert>
#include <cmath>


namespace tglib {

/**
 * @brief A function for transforming ordered edge list representation to incident lists representation.
 *
 * @tparam N The node type of the IncidentLists representation
 * @tparam E The temporal edge type of the OrderedEdgeList
 * @param tgs A temporal graph in ordered edge list representation.
 * @return The temporal graph in incident lists representation.
 */
template<typename N, typename E>
tglib::IncidentLists<N, E> to_incident_lists(tglib::OrderedEdgeList<E> const &tgs) {
    IncidentLists<N, E> g(tgs.getNumberOfNodes(), tgs.getEdges());
    return g;
}

/**
 * A function for transforming incident lists representation to ordered edge list representation.
 * @param tg A temporal graph in incident lists representation.
 * @return The temporal graph in ordered edge list representation.
 */
template<typename N, typename E>
tglib::OrderedEdgeList<E> to_ordered_edge_list(tglib::IncidentLists<N, E> const &tg) {
    // todo nodemapping
    OrderedEdgeList<E> tgs;

    for (auto &n : tg.nodes) {
        for (auto &e : n.outEdges) {
            tgs.edges.push_back(e);
        }
    }

    sort(tgs.edges.begin(), tgs.edges.end(),
              [](auto const &e1, auto const &e2){return e1.t < e2.t;});

    tgs.num_nodes = tg.nodes.size();
    tgs.ti = tg.ti;

    return tgs;
}

/**
 * A function for transforming ordered edge list representation to time-respecting static graph.
 * @param tgs A temporal graph in ordered edge list representation.
 * @return The temporal graph in incident lists representation.
 */
template<typename E>
tglib::TRSGraph to_trs_graph(tglib::OrderedEdgeList<E> const &tgs) {
    TRSGraph trs;
    trs.positionOfEarliestTimeNode.resize(tgs.getNumberOfNodes(), inf);
    trs.positionOfLatestTimeNode.resize(tgs.getNumberOfNodes(), inf);
    trs.numTGNodes = tgs.getNumberOfNodes();
    trs.numEdges = 0;

    std::set<TimeNode> timeNodes;
    std::vector<Time> maxArrivalTimes(tgs.getNumberOfNodes(), -1);
    std::vector<std::set<Time>> startingTimes(tgs.getNumberOfNodes(), std::set<Time>());

    std::map<TimeNode, NodeId> nodeMap;

    NodeId nid = 0;

    for (auto &e : tgs.getEdges()) {
        startingTimes[e.u].insert(e.t);
        if (maxArrivalTimes[e.v] < e.t + e.tt) {
            maxArrivalTimes[e.v] = e.t + e.tt;
        }
    }
    for (size_t i = 0; i < tgs.getNumberOfNodes(); ++i) {
        if (maxArrivalTimes[i] > -1) {
            timeNodes.insert({i, maxArrivalTimes[i]});
            startingTimes[i].insert(maxArrivalTimes[i]);
        }

        auto numberOfStartingTimes = startingTimes[i].size();
        for (Time startingTime : startingTimes[i]){
            TRSNode trsNode;
            trsNode.tn = {i, startingTime};
            trsNode.nid = nid++;

            timeNodes.insert(trsNode.tn);
            nodeMap[trsNode.tn] = trsNode.nid;

            if (numberOfStartingTimes > 1) {
                TRSEdge trsEdge{};
                trsEdge.u = trsNode.nid;
                trsEdge.v = nid;
                trsEdge.tt = 0;
                trsNode.adjList.push_back(trsEdge);
                trs.numEdges++;
            }
            trs.nodes.push_back(trsNode);

            if (numberOfStartingTimes == startingTimes[i].size()) {
                trs.positionOfEarliestTimeNode[i] = trs.nodes.size()-1;
            }

            numberOfStartingTimes--;

            if (numberOfStartingTimes == 0) {
                trs.positionOfLatestTimeNode[i] = trs.nodes.size()-1;
            }

        }
    }
    for (auto &e : tgs.getEdges()) {
        NodeId u = nodeMap[{e.u, e.t}];
        auto it = lower_bound(startingTimes[e.v].begin(), startingTimes[e.v].end(), e.t + e.tt);
        assert(it != startingTimes[e.v].end());
        Time arrTime = *(it);
        NodeId v = nodeMap[{e.v, arrTime}];
        TRSEdge trsEdge{u, v, e.tt};
        trs.nodes[u].adjList.push_back(trsEdge);
        trs.numEdges++;
    }

    trs.ti = tgs.getTimeInterval();

    return trs;
}

/**
 * @brief Transformation to directed line graph.
 * @tparam N The node type of the input graph
 * @param tg The input graph
 * @return The corresponding directed line graph
 */
template<typename N, typename E>
tglib::DirectedLineGraph to_directed_line_graph(tglib::IncidentLists<N, E> const &tg) {
    DirectedLineGraph dlg;

    dlg.nodes.clear();
    dlg.nodes.resize(tg.getNumberOfEdges());
    std::vector<bool> nodeset(tg.getNumberOfEdges(), false);

    std::map<TemporalEdge, EdgeId> eids;
    EdgeId eid = 0;
    for (auto &tgn : tg.getNodes()) {
        for (auto &e: tgn.outEdges) {
            eids[e] = eid++;
        }
    }

    for (auto &tgn : tg.getNodes()) {
        for (auto &e : tgn.outEdges) {
            if (!nodeset.at(eids[e])) {
                DLNode node;
                node.id = eids[e];
                node.e = e;
                dlg.nodes.at(node.id) = node;
                nodeset.at(eids[e]) = true;
            }
            for (auto &f : tg.getNode(e.v).outEdges) {
                if (e.t + e.tt > f.t) continue;
                if (!nodeset.at(eids[f])) {
                    DLNode node;
                    node.id = eids[f];
                    node.e = f;
                    dlg.nodes.at(node.id) = node;
                    nodeset.at(eids[f]) = true;
                }
                auto &m = dlg.nodes.at(eids[f]);
                auto &n = dlg.nodes.at(eids[e]);
                auto de = DLEdge();
                de.u = n.id;
                de.v = m.id;
                de.u_t = n.e.t;
                de.v_t = m.e.t;
                de.tg_u = n.e.u;
                de.tg_w = m.e.v;
                de.tg_v = n.e.v;
                n.outedges.push_back(de);
//                m.inedges.push_back(de);
                dlg.num_edges++;
            }
        }
    }
    dlg.num_tg_nodes = tg.getNumberOfNodes();

    return dlg;
}

/**
 * @brief Transformation to directed line graph.
 * @tparam E The edge type of the input graph
 * @param tgs The input graph
 * @return The corresponding directed line graph
 */
template<class E>
tglib::DirectedLineGraph to_directed_line_graph(tglib::OrderedEdgeList<E> const &tgs){
    auto tg = to_incident_lists<TGNode, E>(tgs);
    return to_directed_line_graph<TGNode>(tg);
}

/**
 * @brief Computes the underlying aggregated graph in which the edges are weighted with the frequency
 * of their temporal contacts.
 * @tparam E The edge type of the input graph
 * @param tgs The input graph
 * @return An edge-weighted static graph as list of static edges
 */
template<typename E>
std::vector<StaticWeightedEdge> to_aggregated_edge_list(tglib::OrderedEdgeList<E> const &tgs) {
    std::map<std::pair<NodeId, NodeId>, long> edges;
    for (auto &e : tgs.getEdges()) {
        edges[{e.u, e.v}] += 1;
    }
    std::vector<StaticWeightedEdge> aggregated_edge_list;
    for (auto &p : edges) {
        aggregated_edge_list.push_back({p.first.first, p.first.second, p.second});
    }
    return aggregated_edge_list;
}

/**
 * @brief Normalizes a temporal graph.
 *
 * Multiple edges will be removed and the resulting edge stream will be chronologically ordered.
 *
 * @tparam E The edge type
 * @param tgs The input graph
 * @param removeLoops If true, self-loops will be removed
 * @return Normalized temporal graph
 */
template<typename E>
OrderedEdgeList<E> normalize(tglib::OrderedEdgeList<E> const &tgs, bool removeLoops) {
    std::set<E> edges;
    for (auto &e : tgs.getEdges()) {
        if (removeLoops && e.u == e.v) continue;
        edges.insert(e);
    }
    std::vector<E> filtered_edges;
    TimeInterval ti = {inf, 0};
    for (auto &e : edges){
        filtered_edges.push_back(e);
        if (e.t < ti.first) ti.first = e.t;
        if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
    }
    OrderedEdgeList<E> normalized_graph(tgs.getNumberOfNodes(), filtered_edges, ti, tgs.getNodeMap());
    return normalized_graph;
}

/**
 * @brief Scales the time stamps by a given factor.
 * @tparam E The temporal edge type.
 * @param tgs The input graph.
 * @param factor The scaling factor.
 * @return The temporal graph with scaled time stamps.
 */
template<typename E>
OrderedEdgeList<E> scale_timestamps(tglib::OrderedEdgeList<E> const &tgs, double factor) {
    std::set<E> edges;
    for (auto &e : tgs.getEdges()) {
        auto f = e;
        f.t = std::round(f.t * factor); //todo maybe floor?
        edges.insert(f);
    }
    std::vector<E> filtered_edges;
    TimeInterval ti = {inf, 0};
    for (auto &e : edges){
        filtered_edges.push_back(e);
        if (e.t < ti.first) ti.first = e.t;
        if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
    }
    OrderedEdgeList<E> normalized_graph(tgs.getNumberOfNodes(), filtered_edges, ti, tgs.getNodeMap());
    return normalized_graph;
}

/**
 * @brief Replaces the transition times with a given value equal for all edges.
 * @tparam E The temporal edge type
 * @param tgs The input graph
 * @param val The new transition time for all edges
 * @return The resulting temporal graph
 */
template<typename E>
OrderedEdgeList<E> unit_transition_times(tglib::OrderedEdgeList<E> const &tgs, Time val) {
    TimeInterval ti = {inf, 0};
    std::vector<E> filtered_edges;
    for (auto e : tgs.getEdges()){
        e.tt = val;
        filtered_edges.push_back(e);
        if (e.t < ti.first) ti.first = e.t;
        if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
    }
    OrderedEdgeList<E> normalized_graph(tgs.getNumberOfNodes(), filtered_edges, ti, tgs.getNodeMap());
    return normalized_graph;
}

} // tglib



#endif //TGLIB_TRANSFORMATIONS_H
