/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include "../../src/algorithms/TemporalPaths.h"
#include "../TemporalGraphsInstances.h"
#include "../../src/core/Transformations.h"

using namespace tglib;

TEST_CASE( "ea path", "[path]" ) {
    auto tgs = getExampleFromPaper();
    auto tg = to_incident_lists<TGNode>(tgs);

    auto path = earliest_arrival_path(tg, 0, 2, tgs.getTimeInterval());
    // todo check
    //    for (auto p : path) {
//        std::cout << p << std::endl;
//    }
}

TEST_CASE( "fastest path", "[path]" ) {
    auto tgs = getExampleFromPaper();
    auto tg = to_incident_lists<TGNode>(tgs);
    auto path = minimum_duration_path(tg, 0, 3, tgs.getTimeInterval());
    // todo check
    //    for (auto p : path) {
//        std::cout << p << std::endl;
//    }
}


TEST_CASE( "shorstest path", "[path]" ) {
    auto tgs = getExampleFromPaper();
    auto tg = to_incident_lists<TGNode>(tgs);
    auto path = minimum_transition_time_path(tg, 0, 3, tgs.getTimeInterval());
    // todo check
    //    for (auto p : path) {
//        std::cout << p << std::endl;
//    }
}

TEST_CASE( "minhop path", "[path]" ) {
    auto tgs = getExampleFromPaper();
    auto tg = to_incident_lists<TGNode>(tgs);
    auto path = minimum_hops_path(tg, 0, 3, tgs.getTimeInterval());
    // todo check
    //    for (auto p : path) {
//        std::cout << p << std::endl;
//    }
}