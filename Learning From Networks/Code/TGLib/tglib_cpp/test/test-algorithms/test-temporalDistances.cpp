/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/TemporalDistances.h"
#include "../../src/core/Transformations.h"

using namespace tglib;

TEST_CASE( "reachability chain", "[reachability]" ) {

    auto tgs = getChain();

    auto arrival_times = tglib::earliest_arrival_times(tgs, 0, tgs.getTimeInterval());

    REQUIRE(arrival_times[0] == 0);
    REQUIRE(arrival_times[1] == 2);
    REQUIRE(arrival_times[2] == 3);
    REQUIRE(arrival_times[3] == 4);
    REQUIRE(arrival_times[4] == 5);
    REQUIRE(arrival_times[5] == 6);
    REQUIRE(arrival_times[6] == 7);
    REQUIRE(arrival_times[7] == 8);

}

TEST_CASE( "duration example", "[duration]" ) {
//    auto tgs = getExampleFromPaper();
    auto tgs = getExample3();
    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto d1 = tglib::minimum_durations(tgs, nid, ti);
        auto d2 = tglib::minimum_durations(trs, nid, ti);
        auto d3 = tglib::minimum_durations(tg, nid, ti);

        REQUIRE(d1.size() == d2.size());
        REQUIRE(d2.size() == d3.size());

        for (size_t i = 0; i < d1.size(); ++i) {
            REQUIRE(d1[i] == d2[i]);
            REQUIRE(d2[i] == d3[i]);
        }
    }
}

TEST_CASE( "ea example", "[ea]" ) {
    auto tgs = getExampleFromPaper();
    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto d1 = tglib::earliest_arrival_times(tgs, nid, ti);
        auto d2 = tglib::earliest_arrival_times(trs, nid, ti);
        auto d3 = tglib::earliest_arrival_times(tg, nid, ti);

        REQUIRE(d1.size() == d2.size());
        REQUIRE(d2.size() == d3.size());

        for (size_t i = 0; i < d1.size(); ++i) {
            REQUIRE(d1[i] == d2[i]);
            REQUIRE(d2[i] == d3[i]);
        }
    }
}

TEST_CASE( "short example", "[shortest]" ) {
    auto tgs = getRandomGraphExample(10,1000);

    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto d1 = tglib::minimum_transition_times(tgs, nid, ti);
//        auto d2 = tglib::minimum_transition_times(trs, nid, ti);
        auto d2 = tglib::minimum_transition_times(tg, nid, ti);

        REQUIRE(d1.size() == d2.size());
//        REQUIRE(d2.size() == d3.size());

        for (size_t i = 0; i < d1.size(); ++i) {
            REQUIRE(d1[i] == d2[i]);
//            REQUIRE(d2[i] == d3[i]);
        }
    }
}

TEST_CASE( "dur infectious", "[duration]" ) {
    auto tgs = getRandomGraphExample(8,40);
    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    NodeId nid = rand() % tgs.getNumberOfNodes();

    auto d1 = tglib::minimum_durations(tgs, nid, ti);
    auto d2 = tglib::minimum_durations(trs, nid, ti);
    auto d3 = tglib::minimum_durations(tg, nid, ti);

    REQUIRE(d1.size() == d2.size());
    REQUIRE(d2.size() == d3.size());

    for (size_t i = 0; i < d1.size(); ++i) {
        REQUIRE(d1[i] == d2[i]);
        REQUIRE(d1[i] == d3[i]);
        REQUIRE(d2[i] == d3[i]);
    }
}

TEST_CASE( "example ua ea2", "[ea]" ) {
    srand (1);
    auto tgs = getRandomGraphExample(4,12);
    auto tg = to_incident_lists<TGNode>(tgs);

    NodeId nid = 3;

    auto d1 = earliest_arrival_times(tg, nid, tgs.getTimeInterval());
    auto d2 = earliest_arrival_times(tgs, nid, tgs.getTimeInterval());
    auto d3 = earliest_arrival_times(tg, nid, tgs.getTimeInterval());

    for (size_t i = 0; i < d1.size(); ++i) {
        REQUIRE(d1[i] == d2[i]);
        REQUIRE(d3[i] == d1[i]);
    }
}


TEST_CASE( "short example hops", "[min_hops]" ) {
    auto tgs = getExampleFromPaper();

    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto d1 = tglib::minimum_hops(tgs, nid, ti);
        auto d2 = tglib::minimum_hops(tg, nid, ti);
        auto d3 = tglib::minimum_hops(trs, nid, ti);

        REQUIRE(d1.size() == d2.size());
        REQUIRE(d2.size() == d3.size());

        for (size_t i = 0; i < d1.size(); ++i) {
            REQUIRE(d1[i] == d2[i]);
            REQUIRE(d2[i] == d3[i]);
        }
    }
}

TEST_CASE( "short example mintrans", "[min_hops]" ) {
    auto tgs = getRandomGraphExample(10,100);
    auto trs = to_trs_graph(tgs);
    auto tg = to_incident_lists<TGNode>(tgs);

    auto ti = tgs.getTimeInterval();

    for (size_t nid = 0; nid < tgs.getNumberOfNodes(); ++nid) {
        auto d1 = tglib::minimum_transition_times(tgs, nid, ti);
        auto d2 = tglib::minimum_transition_times(tg, nid, ti);
        auto d3 = tglib::minimum_transition_times(trs, nid, ti);

        REQUIRE(d1.size() == d2.size());
        REQUIRE(d2.size() == d3.size());

        for (size_t i = 0; i < d1.size(); ++i) {
            REQUIRE(d1[i] == d2[i]);
            REQUIRE(d2[i] == d3[i]);
        }
    }

}
