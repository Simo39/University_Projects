/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/Burstiness.h"
#include "../../src/util/InputOutput.h"

using namespace tglib;

TEST_CASE( "edge_burstiness non-bursty", "[burstiness]" ) {
    auto tgs = getNonBursty(1000);
    auto b = edge_burstiness(tgs, tgs.getTimeInterval());

    for (auto &p : b){
        REQUIRE(p.second < -0.8);
    }
}

TEST_CASE( "edge_burstiness random", "[burstiness]" ) {
    auto tgs = getRandomGraphExample(2, 1000);
    auto b = edge_burstiness(tgs, tgs.getTimeInterval());

    for (auto &p : b) {
        REQUIRE(p.second > -0.5);
        REQUIRE(p.second < 0.5);
    }

}

TEST_CASE( "burstiness example", "[burstiness]" ) {
    auto tgs = load_ordered_edge_list<TemporalEdge>(
            "../../../example_datasets/example_from_paper.tg");

    auto b = edge_burstiness(tgs, tgs.getTimeInterval());

    REQUIRE(b.size() == 2);

    for (auto &p : b) {
        REQUIRE(p.second >= -1.0);
        REQUIRE(p.second <= 1.0);
    }

    auto nb = node_burstiness(tgs, tgs.getTimeInterval());

    REQUIRE(nb.size() == 4);

    for (auto &p : nb) {
        REQUIRE(p >= -1.0);
        REQUIRE(p <= 1.0);
    }

}


TEST_CASE( "edge_burstiness bursty", "[burstiness3]" ) {
    auto tgs = getBursty(1000);
    auto b = edge_burstiness(tgs, tgs.getTimeInterval());

    for (auto &p : b)
        REQUIRE(p.second > 0.8);
}

TEST_CASE("node_burstiness non-bursty", "[node_burstiness1]") {
    auto tgs = getNonBursty(1000);
    auto b = node_burstiness(tgs, tgs.getTimeInterval());

    for (auto &p : b){
        REQUIRE(p < -0.8);
    }
}

TEST_CASE("node_burstiness bursty", "[node_burstiness2]") {
    auto tgs = getBursty(1000);
    auto b = node_burstiness(tgs, tgs.getTimeInterval());

    for (auto &p : b){
        REQUIRE(p > 0.8);
    }
}