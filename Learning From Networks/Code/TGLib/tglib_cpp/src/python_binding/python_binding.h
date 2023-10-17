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

#ifndef TGLIB_PYTHON_BINDING_H
#define TGLIB_PYTHON_BINDING_H

#include <map>
#include "../core/BasicTypes.h"
#include "../core/OrderedEdgeList.h"

PYBIND11_MAKE_OPAQUE(std::vector<double>)
PYBIND11_MAKE_OPAQUE(std::vector<tglib::Time>)
PYBIND11_MAKE_OPAQUE(std::map<std::pair<tglib::NodeId, tglib::NodeId>, double>)

namespace tglib_python_binding {

// Core
void bind_BasicTypes(pybind11::module_ &m);
void bind_OrderedEdgeList(pybind11::module_ &m);
void bind_IncidentLists(pybind11::module_ &m);
void bind_Transformations(pybind11::module_ &m);

// Algorithms
void bind_Burstiness(pybind11::module_ &m);
void bind_TemporalDiameter(pybind11::module_ &m);
void bind_TemporalDistances(pybind11::module_ &m);
void bind_TemporalBetweenness(pybind11::module_ &m);
void bind_TemporalCloseness(pybind11::module_ &m);
void bind_TemporalClusteringCoefficient(pybind11::module_ &m);
void bind_TemporalEfficiency(pybind11::module_ &m);
void bind_TemporalKatz(pybind11::module_ &m);
void bind_TemporalPageRank(pybind11::module_ &m);
void bind_TemporalPaths(pybind11::module_ &m);
void bind_TemporalReachability(pybind11::module_ &m);
void bind_TemporalWalkCentrality(pybind11::module_ &m);
void bind_TopkTemporalCloseness(pybind11::module_ &m);
void bind_TopologicalOverlap(pybind11::module_ &m);

// Util
void bind_InputOutput(pybind11::module_ &m);

} // tglib_python_binding

#endif //TGLIB_PYTHON_BINDING_H
