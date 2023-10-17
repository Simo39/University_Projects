/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <algorithm>
#include "../../src/util/TopkResult.h"

using namespace tglib;
using namespace std;

TEST_CASE( "topkresult top50 out of 1000", "[topkresult]" ) {
    int k = 50;
    int total = 1000;
    TopkResult<double> topkResult(k);
    std::vector<std::pair<NodeId, double>> values;
    for (int i = 0; i < total; ++i) {
        auto r = rand() % 10000;
        values.emplace_back(i, r);
        topkResult.insert(i, r);
        REQUIRE((((i+1) < k && !topkResult.doApproximation()) ||
                 ((i+1) >= k && topkResult.doApproximation())));
    }

    sort(values.begin(), values.end(), [](auto &a, auto &b) {return a.first > b.first;});
    stable_sort(values.begin(), values.end(), [](auto &a, auto &b) {return a.second > b.second;});

    auto result = topkResult.getResults();

    // for comparison
    sort(result.begin(), result.end(), [](auto &a, auto &b) {return a.first > b.first;});
    stable_sort(result.begin(), result.end(), [](auto &a, auto &b) {return a.second > b.second;});

    for (int i = 0; i < k; ++i) {
        REQUIRE(result[i].first == values[i].first);
        REQUIRE(result[i].second == values[i].second);
    }
}
