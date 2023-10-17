/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/TemporalClusteringCoefficient.h"
#include "../../src/core/Transformations.h"

using namespace tglib;
using namespace std;

TEST_CASE( "temporal clustering coefficient chain", "[tcc]" ) {
    auto tgs = getChain();
    auto tg = to_incident_lists<TGNode>(tgs);
    auto tcc = temporal_clustering_coefficient(tg, 2, tg.getTimeInterval());

    REQUIRE(tcc == 0);
}

TEST_CASE( "temporal clustering coefficient K_5", "[tcc]" ) {
    auto tgs = getCompleteGraph();
    auto tg = to_incident_lists<TGNode>(tgs);

    for (size_t i = 0; i < tg.getNumberOfNodes(); ++i) {
        auto tcc = temporal_clustering_coefficient(tg, 2, tg.getTimeInterval());
        REQUIRE(tcc == 1);
    }
}
