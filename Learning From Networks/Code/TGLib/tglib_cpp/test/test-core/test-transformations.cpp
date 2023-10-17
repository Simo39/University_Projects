/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include "catch2/catch_test_macros.hpp"
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/TemporalDistances.h"
#include "../../src/core/Transformations.h"
#include <algorithm>

using namespace tglib;

TEST_CASE( "TRS example", "[trs_example]" ) {
    auto tgs = getExampleFromPaper();

    auto trs = to_trs_graph(tgs);

    REQUIRE(trs.nodes.size() == 9);
    REQUIRE(trs.numEdges == 5+7);

    REQUIRE(trs.nodes[0].adjList.size() == 2);
    REQUIRE(trs.nodes[1].adjList.size() == 2);
    REQUIRE(trs.nodes[2].adjList.size() == 1);
    REQUIRE(trs.nodes[3].adjList.size() == 1);
    REQUIRE(trs.nodes[4].adjList.size() == 2);
    REQUIRE(trs.nodes[5].adjList.empty());
    REQUIRE(trs.nodes[6].adjList.size() == 2);
    REQUIRE(trs.nodes[7].adjList.size() == 2);
    REQUIRE(trs.nodes[8].adjList.empty());

    REQUIRE(trs.nodes[0].adjList[0].u == 0);
    REQUIRE(trs.nodes[0].adjList[0].v == 1);
    REQUIRE(trs.nodes[0].adjList[0].tt == 0);

    REQUIRE(trs.nodes[0].adjList[0].u == 0);
    REQUIRE(trs.nodes[0].adjList[1].v == 6);
    REQUIRE(trs.nodes[0].adjList[1].tt == 5);

}


TEST_CASE( "no normalization", "[filter]" ) {
    auto tgs = getExampleFromPaper();
    auto ntgs = normalize(tgs, true);

    REQUIRE(tgs == ntgs);
}

TEST_CASE( "normalization", "[filter]" ) {
    auto tgs = getExampleFromPaperWithLoopsAndMulitEdges();
    auto tgs2 = getExampleFromPaper();
    auto ntgs = normalize(tgs, true);

    REQUIRE(tgs2 == ntgs);
}

TEST_CASE( "normalization, keep loops", "[filter]" ) {
    auto tgs = getExampleFromPaperWithLoopsAndMulitEdges();
    auto ntgs = normalize(tgs, false);

    REQUIRE(ntgs.getEdges()[2].u == ntgs.getEdges()[2].v);
}

TEST_CASE( "normalization, non sorted rnd graph", "[filter]" ) {
    auto tgs = getRandomGraphNonSorted(100, 1000);
    auto ntgs = normalize(tgs, true);

    REQUIRE(std::is_sorted(ntgs.getEdges().begin(), ntgs.getEdges().end()));
}

TEST_CASE( "scale example 10", "[filter]" ) {
    auto tgs = getExampleFromPaper();
    auto ntgs = scale_timestamps(tgs, 10);

    REQUIRE(tgs.getNumberOfNodes() == ntgs.getNumberOfNodes());
    REQUIRE(tgs.getEdges().size() == ntgs.getEdges().size());
    REQUIRE((tgs.getEdges()[0].t * 10) == ntgs.getEdges()[0].t);
}

TEST_CASE( "unit transition times", "[filter]" ) {
    auto tgs = normalize(getRandomGraphNonSorted(100, 1000), false);
    auto ntgs = unit_transition_times(tgs, 1);

    for (auto &e : ntgs.getEdges()) {
        REQUIRE(e.tt == 1);
    }
}