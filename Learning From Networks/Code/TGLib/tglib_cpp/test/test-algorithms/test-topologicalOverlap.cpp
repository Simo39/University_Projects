/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/TopologicalOverlap.h"
#include "../../src/core/Transformations.h"

using namespace tglib;

TEST_CASE( "no topological overlap example", "[notopo]" ) {
    auto tgs = getNoTopologicalOverlap(200);
    auto tg = to_incident_lists<TGNode>(tgs);
    auto topo = topological_overlap(tg, 0, tgs.getTimeInterval());
    REQUIRE(topo == 0);
}

TEST_CASE( "full topological overlap example", "[fulltopo]" ) {
    auto tgs = getFullTopologicalOverlap(10);
    auto tg = to_incident_lists<TGNode>(tgs);
    auto topo = topological_overlap(tg, 0, tgs.getTimeInterval());
    REQUIRE(topo == 1);
}
