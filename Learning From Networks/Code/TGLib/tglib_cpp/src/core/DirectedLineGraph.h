/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file DirectedLineGraph.h
 *  @brief Contains the DirectedLineGraph representation data structure for temporal graphs
 */

#ifndef TGLIB_DIRECTEDLINEGRAPH_H
#define TGLIB_DIRECTEDLINEGRAPH_H

#include <vector>
#include "BasicTypes.h"

namespace tglib {

/**
 * @brief Edge type for directed line graph.
 *
 * An edge in a directed line graph represents a temporal path of length two
 * in the temporal graph, i.e., ((tg_u, tg_v, u_t), (tg_v, tg_w, v_t))
 */
struct DLEdge {
    /**
     * @brief Id of the tail in the dlg
     */
    NodeId u;

    /**
     * @brief Id of the head in the dlg
     */
    NodeId v;

    /**
     * @brief Id of the tail of the temporal first edge
     */
    NodeId tg_u;

    /**
     * @brief Id of the head of the temporal first edge
     */
    NodeId tg_v;

    /**
     * @brief Id of the head of the temporal second edge
     */
    NodeId tg_w;

    /**
     * @brief Availability time of the first edge
     */
    Time u_t;

    /**
     * @brief Availability time of the second edge
     */
    Time v_t;
};

/**
 * @brief Node type for directed line graph
 */
struct DLNode {
    /**
     * The
     */
    NodeId id;

    /**
     * @brief The corresponding temporal edge
     */
    TemporalEdge e;

    /**
     * @brief The incident edges
     */
    std::vector<DLEdge> outedges;
};

/**
 * @brief The directed line graph data structure
 */
struct DirectedLineGraph {
    /**
     * @brief Number of nodes in the corresponding temporal graph
     */
    unsigned long num_tg_nodes{};

    /**
     * @brief Number of edges in the DLG
     */
    unsigned long num_edges{};

    /**
     * @brief The nodes of the DLG
     */
    std::vector<DLNode> nodes;
};


} // tglib


#endif //TGLIB_DIRECTEDLINEGRAPH_H
