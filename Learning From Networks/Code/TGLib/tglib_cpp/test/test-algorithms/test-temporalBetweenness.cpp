/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <iostream>
#include "../../src/algorithms/TemporalBetweenness.h"
#include "../../src/util/InputOutput.h"
#include "../TemporalGraphsInstances.h"

using namespace tglib;

TEST_CASE( "edge betweenness", "[edge_betweenness]" ) {
    auto tgs = getBetweennessTest();
    auto dlg = to_directed_line_graph<TemporalEdge>(tgs);

    auto b = temporal_edge_betweenness(dlg);

    for (size_t i = 0; i < tgs.getNumberOfNodes(); ++i) {
        REQUIRE(b[i] >= 0);
    }
}