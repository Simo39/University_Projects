/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/TemporalCloseness.h"
#include "../../src/algorithms/TopkTemporalCloseness.h"
#include "../../src/core/Transformations.h"

using namespace tglib;
using namespace std;

TEST_CASE( "value chain node 0", "[temporal_closeness]" ) {
    auto tgs = getChain();
    auto closeness = temporal_closeness(tgs, 0, tgs.getTimeInterval(), Distance_Type::Fastest);
    REQUIRE(closeness >= 2.5928);
}


TEST_CASE( "value chain topk", "[temporal_closeness]" ) {
    auto tgs = getChain();
    auto tg = to_incident_lists<TGNode>(tgs);
    auto closeness = compute_topk_closeness<TGNode>(tg, 2, tgs.getTimeInterval(), Distance_Type::Fastest);
    REQUIRE(closeness[0].first == 0);
    REQUIRE(closeness[0].second >= 2.5928);
}

TEST_CASE( "closeness example", "[temporal_closeness]" ) {
    auto tgs = getExampleFromPaper();
    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto d1 = temporal_closeness(tgs, nid, ti, Distance_Type::Fastest);
        auto d2 = temporal_closeness(trs, nid, ti);
        auto d3 = tglib::temporal_closeness(tg, nid, ti, Distance_Type::Fastest);

        REQUIRE(d1 == d2);
        REQUIRE(d2 == d3);

    }
}
