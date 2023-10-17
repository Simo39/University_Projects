# Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
#
# This file is part of TGLib which is released under MIT license.
# See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
# for full license details.

cmake_minimum_required(VERSION 3.4...3.18)
project(tglib)

set(TO_DOCUMENT

        src/core/BasicTypes.h
        src/core/OrderedEdgeList.h

        src/core/AggregatedGraph.h
        src/core/IncidentLists.h
        src/core/TRSGraph.h
        src/core/DirectedLineGraph.h
        src/core/AggregatedGraph.h
        src/core/Transformations.h

        src/util/LabelPQ.h
        src/util/TopkResult.h
        src/util/UtilFunctions.h

        src/algorithms/TemporalBetweenness.h
        src/algorithms/TemporalClusteringCoefficient.h
        src/algorithms/TemporalKatz.h
        src/algorithms/TemporalPageRank.h
        src/algorithms/Burstiness.h
        src/algorithms/TemporalDistances.h

        src/algorithms/TemporalCloseness.h
        src/algorithms/TemporalWalkCentrality.h
        src/algorithms/TemporalEfficiency.h
        src/algorithms/TemporalDiameter.h
        src/algorithms/TopologicalOverlap.h
        src/algorithms/TemporalReachability.h

        src/util/InputOutput.h

        PARENT_SCOPE)

add_subdirectory(python_binding)
