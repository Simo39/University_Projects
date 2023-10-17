/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file AggregatedGraph.h
 *  @brief Contains types for the aggregated graph representation.
 */

#ifndef TGLIB_AGGREGATEDGRAPH_H
#define TGLIB_AGGREGATEDGRAPH_H

#include "BasicTypes.h"

namespace tglib {

/**
 * @brief A simple weigted (non-temporal) static edge.
 */
struct StaticWeightedEdge {
    /**
     * @brief The tail
     */
    NodeId u;

    /**
     * @brief The head
     */
    NodeId v;

    /**
     * @brief The weight
     */
    int64_t weight;
};

} // tglib

#endif //TGLIB_AGGREGATEDGRAPH_H
