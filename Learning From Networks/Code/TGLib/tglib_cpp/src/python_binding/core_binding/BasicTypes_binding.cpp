/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file BasicTypes_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include <pybind11/stl_bind.h>

#include "../../core/BasicTypes.h"
#include <map>

namespace tglib_python_binding {

using namespace tglib;

void bind_BasicTypes(pybind11::module_ &m) {
    pybind11::bind_vector<std::vector<double>>(m, "VectorDouble");
    pybind11::bind_vector<std::vector<Time>>(m, "VectorTime");
    pybind11::bind_vector<std::vector<NodeId>>(m, "VectorNodeId");
    pybind11::bind_vector<std::vector<TemporalEdge>>(m, "VectorTemporalEdge");
    pybind11::bind_map<std::map<std::pair<NodeId, NodeId>, double>>(m, "MapPairNodeIdsDouble");
    pybind11::bind_vector<std::vector<std::pair<tglib::NodeId, double>>>(m, "VectorNodeIdDoublePair");

    pybind11::enum_<Distance_Type>(m, "Distance_Type", "An enum type for distance types.")
            .value("Fastest", Distance_Type::Fastest)
            .value("Earliest_Arrival", Distance_Type::Earliest_Arrival)
            .value("Minimum_Transition_Times", Distance_Type::Minimum_Transition_Times)
            .value("Minimum_Hops", Distance_Type::Minimum_Hops)
            .value("Latest_Departure", Distance_Type::Latest_Departure)
            ;

    pybind11::class_<TemporalGraphStatistics>(m, "TemporalGraphStatistics")
            .def(pybind11::init<>())
            .def_readwrite("numberOfNodes", &TemporalGraphStatistics::numberOfNodes)
            .def_readwrite("numberOfEdges", &TemporalGraphStatistics::numberOfEdges)
            .def_readwrite("numberOfStaticEdges", &TemporalGraphStatistics::numberOfStaticEdges)
            .def_readwrite("numberOfTimeStamps", &TemporalGraphStatistics::numberOfTimeStamps)
            .def_readwrite("numberOfTransitionTimes", &TemporalGraphStatistics::numberOfTransitionTimes)
            .def_readwrite("minimalTimeStamp", &TemporalGraphStatistics::minimalTimeStamp)
            .def_readwrite("maximalTimeStamp", &TemporalGraphStatistics::maximalTimeStamp)
            .def_readwrite("minimalTransitionTime", &TemporalGraphStatistics::minimalTransitionTime)
            .def_readwrite("maximalTransitionTime", &TemporalGraphStatistics::maximalTransitionTime)
            .def_readwrite("maxTemporalInDegree", &TemporalGraphStatistics::maxTemporalInDegree)
            .def_readwrite("minTemporalInDegree", &TemporalGraphStatistics::minTemporalInDegree)
            .def_readwrite("maxTemporalOutDegree", &TemporalGraphStatistics::maxTemporalOutDegree)
            .def_readwrite("minTemporalOutDegree", &TemporalGraphStatistics::minTemporalOutDegree)
            .def("__str__", &TemporalGraphStatistics::toString)
            ;
}

} // tglib_python_binding