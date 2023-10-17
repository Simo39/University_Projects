/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include <algorithm>
#include "../TemporalGraphsInstances.h"
#include "../../src/algorithms/TemporalCloseness.h"
#include "../../src/algorithms/TopkTemporalCloseness.h"
#include "../../src/core/Transformations.h"

using namespace tglib;
using namespace std;


TEST_CASE( "example topk random", "[temporal_closeness]" ) {
    srand (1);
    auto tgs = getRandomGraphExample(100,1000);

    auto tg = to_incident_lists<TGNode>(tgs);

    auto topk = compute_topk_closeness(tg, 10, tgs.getTimeInterval(), Distance_Type::Fastest);
    auto closeness = temporal_closeness(tgs, tgs.getTimeInterval(), Distance_Type::Fastest);
    auto closeness2 = temporal_closeness(tg, tgs.getTimeInterval(), Distance_Type::Fastest);

    sort(closeness.begin(), closeness.end(), greater<>());
    sort(closeness2.begin(), closeness2.end(), greater<>());

    for (size_t i = 0; i < topk.size(); ++i) {
        REQUIRE(topk[i].second == Catch::Approx(closeness[i]));
        REQUIRE(topk[i].second == Catch::Approx(closeness2[i]));
    }
}

TEST_CASE( "example topk shortest", "[temporal_closeness]" ) {
    srand (1);
    auto tgs = getRandomGraphExample(100,1000);

    auto tg = to_incident_lists<TGNode>(tgs);

    auto topk = compute_topk_closeness(tg, 10, tgs.getTimeInterval(), Distance_Type::Minimum_Transition_Times);
    auto closeness = temporal_closeness(tgs, tgs.getTimeInterval(), Distance_Type::Minimum_Transition_Times);
    auto closeness2 = temporal_closeness(tg, tgs.getTimeInterval(), Distance_Type::Minimum_Transition_Times);

    sort(closeness.begin(), closeness.end(), greater<>());
    sort(closeness2.begin(), closeness2.end(), greater<>());

    for (size_t i = 0; i < topk.size(); ++i) {
        REQUIRE(topk[i].second == Catch::Approx(closeness[i]));
        REQUIRE(topk[i].second == Catch::Approx(closeness2[i]));
    }
}