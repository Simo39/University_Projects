/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file OrderedEdgeList.h
 *  @brief Contains the ordered edge list representation data structure for temporal graphs
 */

#ifndef CPP_TEMPORALGRAPHSTREAM_H
#define CPP_TEMPORALGRAPHSTREAM_H

#include <limits>
#include <utility>
#include <vector>
#include <string>
#include <unordered_set>
#include <unordered_map>
#include <set>
#include "BasicTypes.h"


namespace tglib {

/**
 * @brief The ordered edge list contains all temporal edges in chronological order
 */
template<typename E>
class OrderedEdgeList {

public:

    /**
     * @brief Default constructor
     */
    OrderedEdgeList() = default;

    /**
     * @brief Constructor
     * @param num_nodes_ The number of nodes
     * @param edges_ The temporal edges
     * @param ti_ The time interval
     */
    OrderedEdgeList(size_t num_nodes_, std::vector<E> edges_, TimeInterval ti_) :
        num_nodes(num_nodes_), edges(edges_), ti(std::move(ti_)){};

    /**
     * @brief Constructor
     * @param num_nodes_ The number of nodes
     * @param edges_ The temporal edges
     * @param ti_ The time interval
     * @param node_mapping_ The node mapping
     */
    OrderedEdgeList(size_t num_nodes_, std::vector<E> edges_, TimeInterval ti_,
                        std::unordered_map<NodeId, NodeId> node_mapping_) :
        num_nodes(num_nodes_), edges(edges_), ti(std::move(ti_)){
        node_map = std::move(node_mapping_);
        reverse_node_map.resize(node_map.size(), 0);
        for (auto p : node_map) {
            reverse_node_map[p.second] = p.first;
        }
    };

    /**
     * @brief Getter for number of nodes
     * @return The number of nodes.
     */
    [[nodiscard]] size_t getNumberOfNodes() const {
        return num_nodes;
    }

     /**
     * @brief Getter for number of edges
     * @return The number of edges.
     */
    [[nodiscard]] size_t getNumberOfEdges() const {
        return edges.size();
    }

    /**
     * @brief Getter for edges
     * @return Const reference to edges.
     */
    const std::vector<E> &getEdges() const {
        return edges;
    }

    /**
     * @brief Getter for number of the time interval spanned by the temporal graph
     * @return The time interval.
     */
    [[nodiscard]] TimeInterval getTimeInterval() const {
        return ti;
    }

    /**
      * @brief Getter for mapping from new ids to the original ids used in input file
      * @return The mapping
      */
    const std::vector<NodeId> &getReverseNodeMap() const {
        return reverse_node_map;
    }

    /**
      * @brief Getter for mapping from new ids to the original ids used in input file
      * @return The mapping
      */
    const std::unordered_map<NodeId, NodeId> &getNodeMap() const {
        return node_map;
    }



private:

    /**
     * @brief The number of nodes.
     */
    size_t num_nodes{};

    /**
     * @brief The chronological ordered temporal edges. Ties are broken arbitrarily.
     */
    std::vector<E> edges;

    /**
     * @brief The time interval spanned by the temporal graph
     */
     TimeInterval ti;

     /**
      * @brief The mapping from new ids to the original ids used in input file
      */
     std::vector<NodeId> reverse_node_map;

     /**
       * @brief The mapping from the original ids used in input file to the new node ids in [0, num_nodes-1]
       */
     std::unordered_map<NodeId, NodeId> node_map;

};

/**
 * @brief == operator OrderedEdgeList<E>
 * @tparam E
 * @param e1
 * @param e2
 * @return
 */
template<typename E>
inline bool operator==(const OrderedEdgeList<E> &e1, const OrderedEdgeList<E> &e2){
    return e1.getNumberOfNodes() == e2.getNumberOfNodes() &&
        e1.getEdges() == e2.getEdges() &&
        e1.getTimeInterval() == e2.getTimeInterval();
}

/**
 * @brief != operator OrderedEdgeList<E>
 * @tparam E
 * @param e1
 * @param e2
 * @return
 */
template<typename E>
inline bool operator!=(const OrderedEdgeList<E> &e1, const OrderedEdgeList<E> &e2){
    return !(e1 == e2);
}


// todo add edge, remove edge
/**
 * @brief Computes the basic statistics of a temporal graph
 * @tparam E The edge type
 * @param tgs The temporal graph
 * @return The basic statistics
 */
template<typename E>
TemporalGraphStatistics get_statistics(OrderedEdgeList<E> const &tgs) {
    TemporalGraphStatistics statistics{};
    statistics.minTemporalInDegree = inf;
    statistics.minTemporalOutDegree = inf;
    statistics.maxTemporalInDegree = 0;
    statistics.maxTemporalOutDegree = 0;
    statistics.maximalTimeStamp = 0;
    statistics.minimalTimeStamp = inf;
    statistics.maximalTransitionTime = 0;
    statistics.minimalTransitionTime = inf;

    std::vector<long> inDegree(tgs.getNumberOfNodes(), 0);
    std::vector<long> outDegree(tgs.getNumberOfNodes(), 0);
    std::unordered_set<Time> times;
    std::unordered_set<Time> transition_times;
    std::set<std::pair<NodeId, NodeId>> static_edges;

    for (auto &e : tgs.getEdges()){
        inDegree[e.v]++;
        outDegree[e.u]++;
        times.insert(e.t);
        transition_times.insert(e.tt);
        static_edges.insert({e.u, e.v});

        if (statistics.maximalTimeStamp < e.t) statistics.maximalTimeStamp = e.t;
        if (statistics.minimalTimeStamp > e.t) statistics.minimalTimeStamp = e.t;
        if (statistics.maximalTransitionTime < e.tt) statistics.maximalTransitionTime = e.tt;
        if (statistics.minimalTransitionTime > e.tt) statistics.minimalTransitionTime = e.tt;
    }

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        if (statistics.minTemporalInDegree > inDegree[nid]) statistics.minTemporalInDegree = inDegree[nid];
        if (statistics.minTemporalOutDegree > outDegree[nid]) statistics.minTemporalOutDegree = outDegree[nid];
        if (statistics.maxTemporalInDegree < inDegree[nid])  statistics.maxTemporalInDegree = inDegree[nid];
        if (statistics.maxTemporalOutDegree < outDegree[nid])  statistics.maxTemporalOutDegree = outDegree[nid];
    }

    statistics.numberOfNodes = tgs.getNumberOfNodes();
    statistics.numberOfEdges = tgs.getEdges().size();
    statistics.numberOfStaticEdges = static_edges.size();
    statistics.numberOfTimeStamps = times.size();
    statistics.numberOfTransitionTimes = transition_times.size();

    return statistics;
}

} // tglib

#endif //CPP_TEMPORALGRAPHSTREAM_H
