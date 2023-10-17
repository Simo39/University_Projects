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
#include "../../src/algorithms/TemporalDiameter.h"
#include "../../src/core/Transformations.h"

using namespace tglib;
using namespace std;

TEST_CASE( "temporal diameter", "[temporal_diameter]" ) {
    auto tgs = getChain();
    auto tg = to_incident_lists<TGNode>(tgs);

    auto efficiency = temporal_diameter<OrderedEdgeList<TemporalEdge>>
            (tgs, tgs.getTimeInterval(), Distance_Type::Fastest);

    auto efficiency2 = temporal_diameter<IncidentLists<TGNode, TemporalEdge>>
            (tg, tgs.getTimeInterval(), Distance_Type::Fastest);

    REQUIRE(efficiency == efficiency2);
}

