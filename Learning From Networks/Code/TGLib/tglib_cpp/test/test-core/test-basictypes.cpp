/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include "../../src/core/BasicTypes.h"
#include "../../src/core/OrderedEdgeList.h"
#include "../../src/core/TRSGraph.h"
#include "../TemporalGraphsInstances.h"
#include <set>
#include <algorithm>

using namespace tglib;

TEST_CASE( "temporal edge compare", "[tge_cmp]" ) {
    TemporalEdge e1{0,1,1,1};
    TemporalEdge e2{0,1,2,1};

    REQUIRE(e1 < e2);
    REQUIRE(e1 != e2);
    REQUIRE(e1 == e1);
}

TEST_CASE( "temporal edge set", "[tge_set]" ) {
    TemporalEdge e0{0,1,1,1};
    TemporalEdge e1{0,1,1,1};
    TemporalEdge e2{0,1,2,1};
    TemporalEdge e3{0,2,2,1};
    TemporalEdge e4{0,2,2,2};
    TemporalEdge e5{1,2,2,2};

    std::set<TemporalEdge> edges;
    edges.insert(e0);
    edges.insert(e1);
    edges.insert(e2);
    edges.insert(e3);
    edges.insert(e4);
    edges.insert(e5);

    REQUIRE(edges.size() == 5);
}

TEST_CASE( "temporal edge sort", "[tge_sort]" ) {
    TemporalEdge e0{0,1,4,1};
    TemporalEdge e1{0,1,98,1};
    TemporalEdge e2{0,1,2,1};
    TemporalEdge e3{0,2,1,1};
    TemporalEdge e4{0,2,9,2};
    TemporalEdge e5{1,2,9,2};
    TemporalEdge e6{1,2,2,2};

    std::vector<TemporalEdge> edges;
    edges.push_back(e0);
    edges.push_back(e1);
    edges.push_back(e2);
    edges.push_back(e3);
    edges.push_back(e4);
    edges.push_back(e5);
    edges.push_back(e6);

    std::sort(edges.begin(), edges.end());

    REQUIRE(edges[0].t < edges[1].t);
    REQUIRE(edges[1].t <= edges[2].t);
}


TEST_CASE( "copy", "[copy]" ) {
    auto tgs = getRandomGraphExample(1000,10000);

    auto cp = tgs;

    REQUIRE(tgs.getNumberOfNodes() == cp.getNumberOfNodes());
    REQUIRE(tgs.getTimeInterval() == cp.getTimeInterval());

    for (size_t i = 0; i < tgs.getEdges().size(); ++i) {
        REQUIRE(tgs.getEdges()[i] == cp.getEdges()[i]);
    }
}