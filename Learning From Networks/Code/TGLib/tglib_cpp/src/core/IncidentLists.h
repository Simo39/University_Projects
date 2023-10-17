/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file IncidentLists.h
 *  @brief Contains the incident lists representation data structure for temporal graphs
 */

#ifndef TGLIB_INCIDENTLISTS_H
#define TGLIB_INCIDENTLISTS_H


#include "OrderedEdgeList.h"


namespace tglib {

/**
 * @brief Node type for incident list representation
 *
 * This is the full generic type that allows using customized edge types, e.g., weighted temporal edges.
 *
 * @tparam E the temporal edge type
 */
template<typename E>
struct TGNodeT {
    /**
     * @brief The node id.
     */
    NodeId id{};
    /**
     * @brief The outgoing temporal edges.
     */
    std::vector<E> outEdges;
};

/**
 * @brief Node type for incident list representation
 *
 * The default type used in the implemented algorithms.
 */
using TGNode = TGNodeT<TemporalEdge>;

/**
 * @brief Type for incident list representation of temporal graphs
 *
 * The incident lists representation holds a vector of its nodes.
 * Each node has a vector of the outgoing temporal edges.
 */
template<typename N, typename E>
class IncidentLists {

public:

    /**
     * @ default constructor
     */
    IncidentLists() = default;

    /**
     * @brief Constructs object from vector of edges
     * @param numberOfNodes The number of nodes
     * @param edges The edges
     */
    explicit IncidentLists(NodeId numberOfNodes, std::vector<E> const &edges) {
        addNodes(numberOfNodes);
        ti = {inf, 0};
        for (auto &e : edges) {
            addEdge(e);
            if (e.t < ti.first) ti.first = e.t;
            if (e.t + e.tt > ti.second) ti.second = e.t + e.tt;
        }
    }

    /**
     * @brief Getter for number of nodes
     * @return The number of nodes
     */
    [[nodiscard]] size_t getNumberOfNodes() const {
        return nodes.size();
    }

    /**
     * @brief Getter for number of edges
     * @return The number of edges
     */
    [[nodiscard]] size_t getNumberOfEdges() const {
        return num_edges;
    }

    /**
     * @brief Getter for the time interval
     * @return The time interval
     */
    [[nodiscard]] TimeInterval getTimeInterval() const {
        return ti;
    }

    /**
     * @brief Getter for a node
     * @param nid the nodes id
     * @return The node with node id nid
     */
    const N& getNode(size_t nid) const {
        return nodes[nid];
    }

    /**
     * @brief Getter for all nodes
     * @return The nodes
     */
    const std::vector<N> &getNodes() const {
        return nodes;
    }


private:

    /**
     * @brief The nodes
     */
    std::vector<N> nodes;

    /**
     * @brief The number of edges
     */
    EdgeId num_edges{};

    /**
     * @brief The time interval spanned by the temporal graph
     */
    TimeInterval ti;

    /**
    * @brief Adds num nodes with ids 0 to num-1
    * @param num the number of nodes to add
    */
    void addNodes(size_t num) {
        for (size_t i = 0; i < num; ++i) {
            N node;
            node.id = i;
            nodes.push_back(node);
        }
    }

    /**
     * Adds a new edge
     * @param e The edge to add
     */
    void addEdge(E e) {
        nodes[e.u].outEdges.push_back(e);
        num_edges++;
    }

};

} // tglib

// todo add edge, remove edge


#endif //TGLIB_INCIDENTLISTS_H
