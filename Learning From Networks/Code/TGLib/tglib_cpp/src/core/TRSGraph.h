/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file TRSGraph.h
 *  @brief Contains the time-respecting static graph representation type.
 */


#ifndef TGLIB_TRSGRAPH_H
#define TGLIB_TRSGRAPH_H

#include <vector>
#include "BasicTypes.h"


namespace tglib {

/**
 * @brief A struct for edges in the time-respecting static graph representation.
 */
struct TRSEdge {
    /**
     * @brief The tail.
     */
    NodeId u;

    /**
     * @brief The head.
     */
    NodeId v;

    /**
     * @brief The traversal time
     */
    Time tt;
};

/**
 * @brief Time-nodes representing a node in the temporal graph at a specific time.
 */
using TimeNode = std::pair<NodeId, Time>;

/**
 * @brief A struct for nodes in the time-respecting static graph representation.
 */
struct TRSNode {
    /**
     * @brief The node id.
     */
    NodeId nid;

    /**
     * @brief The corresponding time-node.
     */
    TimeNode tn;

    /**
     * @brief The incident edges.
     */
    std::vector<TRSEdge> adjList;
};

/**
 * @brief The time-respecing static graph is a DAG that can be used for temporal distance computations.
 */
struct TRSGraph {
    /**
     * @brief The nodes.
     */
    std::vector<TRSNode> nodes;

    /**
     * @brief positionOfEarliestTimeNode[i] is position of the earliest node in nodes such that the
     * corresponding vertex in the temporal graph is i.
     */
    std::vector<long> positionOfEarliestTimeNode;

    /**
     * @brief positionOfLatestTimeNode[i] is position of the latest node in nodes such that the
     * corresponding vertex in the temporal graph is i.
     */
    std::vector<long> positionOfLatestTimeNode;

    /**
     * @brief The number of edges.
     */
    long numEdges;

    /**
     * @brief The number of nodes in the corresponding temporal graph.
     */
    long numTGNodes;

    /**
     * @brief The time interval spanned by the temporal graph
     */
    TimeInterval ti;
};

} // tglib

#endif //TGLIB_TRSGRAPH_H
