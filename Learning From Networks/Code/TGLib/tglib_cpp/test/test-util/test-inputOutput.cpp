/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include "../../src/util/InputOutput.h"

using namespace tglib;
using namespace std;

TEST_CASE( "load example", "[IO]" ) {
    auto tgs = load_ordered_edge_list<TemporalEdge>(
            "../../../example_datasets/example_from_paper.tg");

    REQUIRE(tgs.getNumberOfNodes() == 4);
    REQUIRE(tgs.getNumberOfEdges() == 7);

    auto &nm = tgs.getReverseNodeMap();
    NodeId u = nm[tgs.getEdges()[0].u];
    NodeId v = nm[tgs.getEdges()[0].v];
    TemporalEdge e {u, v, 1, 5};
    REQUIRE(e == TemporalEdge{0, 3, 1, 5});
}