/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */


/**
 * @file python_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>

#include "python_binding.h"

namespace py = pybind11;

using namespace tglib_python_binding;

PYBIND11_MODULE(pytglib, m) {
// Core
    bind_BasicTypes(m);
    bind_OrderedEdgeList(m);
    bind_IncidentLists(m);
    bind_Transformations(m);

// Algorithms
    bind_Burstiness(m);
    bind_TemporalDiameter(m);
    bind_TemporalDistances(m);
    bind_TemporalBetweenness(m);
    bind_TemporalCloseness(m);
    bind_TemporalClusteringCoefficient(m);
    bind_TemporalEfficiency(m);
    bind_TemporalKatz(m);
    bind_TemporalPageRank(m);
    bind_TemporalPaths(m);
    bind_TemporalReachability(m);
    bind_TemporalWalkCentrality(m);
    bind_TopkTemporalCloseness(m);
    bind_TopologicalOverlap(m);

// Util
    bind_InputOutput(m);
}