/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file IncidentLists_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include "../../core/IncidentLists.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_IncidentLists(pybind11::module_ &m) {

    pybind11::class_<TGNode>(m, "TGNode")
            .def(pybind11::init<>())
            .def_readwrite("id", &TGNode::id)
            .def_readwrite("outEdges", &TGNode::outEdges)
            ;

    pybind11::class_<IncidentLists<TGNode, TemporalEdge>>(m, "IncidentLists")
            .def(pybind11::init<>())
            .def(pybind11::init<NodeId, const std::vector<TemporalEdge> &>())
            .def("getNode", &IncidentLists<TGNode, TemporalEdge>::getNode)
            .def("getNodes", &IncidentLists<TGNode, TemporalEdge>::getNodes)
            .def("getNumberOfNodes", &IncidentLists<TGNode, TemporalEdge>::getNumberOfNodes)
            .def("getNumberOfEdges", &IncidentLists<TGNode, TemporalEdge>::getNumberOfEdges)
            .def("getTimeInterval", &IncidentLists<TGNode, TemporalEdge>::getTimeInterval)
            ;
}

} // tglib_python_binding