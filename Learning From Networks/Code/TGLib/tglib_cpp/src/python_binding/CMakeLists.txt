# Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
#
# This file is part of TGLib which is released under MIT license.
# See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
# for full license details.

cmake_minimum_required(VERSION 3.4...3.18)
project(tglib)

set(PYTHON_BINDING_SOURCES
        python_binding.cpp

        core_binding/AggregatedGraph_binding.cpp
        core_binding/BasicTypes_binding.cpp
        core_binding/DirectedLineGraph_binding.cpp
        core_binding/OrderedEdgeList_binding.cpp
        core_binding/IncidentLists_binding.cpp
        core_binding/Transformations_binding.cpp

        algorithms_binding/Burstiness_binding.cpp
        algorithms_binding/TemporalDiameter_binding.cpp
        algorithms_binding/TemporalDistances_binding.cpp
        algorithms_binding/TemporalPaths_binding.cpp
        algorithms_binding/TemporalBetweenness_binding.cpp
        algorithms_binding/TemporalCloseness_binding.cpp
        algorithms_binding/TemporalClusteringCoefficient_binding.cpp
        algorithms_binding/TemporalEfficiency_binding.cpp
        algorithms_binding/TemporalKatz_binding.cpp
        algorithms_binding/TemporalPageRank_binding.cpp
        algorithms_binding/TopkTemporalCloseness_binding.cpp
        algorithms_binding/TopologicalOverlap_binding.cpp
        algorithms_binding/TemporalWalkCentrality_binding.cpp
        algorithms_binding/TemporalReachability_binding.cpp

        util_binding/InputOutput_binding.cpp
        )

pybind11_add_module(pytglib ${PYTHON_BINDING_SOURCES})
