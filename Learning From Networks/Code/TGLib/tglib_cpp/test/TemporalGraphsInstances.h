/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#ifndef TGPR_TEMPORALGRAPHSINSTANCES_H
#define TGPR_TEMPORALGRAPHSINSTANCES_H

#include "../src/core/OrderedEdgeList.h"

tglib::OrderedEdgeList<tglib::TemporalEdge> getTemporalGraphExample1();
tglib::OrderedEdgeList<tglib::TemporalEdge> getTemporalGraphExample2();

tglib::OrderedEdgeList<tglib::TemporalEdge> getRandomGraphExample(int nodes, int edges);

tglib::OrderedEdgeList<tglib::TemporalEdge> getChain();

tglib::OrderedEdgeList<tglib::TemporalEdge> getNonBursty(int edges);

tglib::OrderedEdgeList<tglib::TemporalEdge> getBursty(int edges);

tglib::OrderedEdgeList<tglib::TemporalEdge> getExampleFromPaper();

tglib::OrderedEdgeList<tglib::TemporalEdge> getExample3();

tglib::OrderedEdgeList<tglib::TemporalEdge> getBetweennessTest();

tglib::OrderedEdgeList<tglib::TemporalEdge> getNoTopologicalOverlap(int edges);

tglib::OrderedEdgeList<tglib::TemporalEdge> getFullTopologicalOverlap(int edges);

tglib::OrderedEdgeList<tglib::TemporalEdge> getCompleteGraph();

tglib::OrderedEdgeList<tglib::TemporalEdge> getExampleFromPaperWithLoopsAndMulitEdges();

tglib::OrderedEdgeList<tglib::TemporalEdge> getRandomGraphNonSorted(int nodes, int num_edges);


#endif //TGPR_TEMPORALGRAPHSINSTANCES_H
