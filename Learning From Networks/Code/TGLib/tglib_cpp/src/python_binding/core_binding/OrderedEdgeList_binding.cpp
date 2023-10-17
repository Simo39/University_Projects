/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file OrderedEdgeList_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include <pybind11/detail/common.h>
#include "../../core/OrderedEdgeList.h"
#include "../python_binding.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_OrderedEdgeList(pybind11::module_ &m) {
    pybind11::class_<TemporalEdge>(m, "TemporalEdge", "The basic temporal edge type.")
            .def(pybind11::init<>())
            .def_readwrite("u", &TemporalEdge::u, "Tail of the edge.")
            .def_readwrite("v", &TemporalEdge::v, "Head of the edge.")
            .def_readwrite("t", &TemporalEdge::t, "Time stamp of the edge.")
            .def_readwrite("tt", &TemporalEdge::tt, "Traversal time of the edge.")
            .def("__str__", &to_string)
            ;

    pybind11::class_<OrderedEdgeList<TemporalEdge>>(m, "OrderedEdgeList")
            .def(pybind11::init<>())
            .def("getNumberOfNodes", &OrderedEdgeList<TemporalEdge>::getNumberOfNodes, "returns the number of nodes")
            .def("getNumberOfEdges", &OrderedEdgeList<TemporalEdge>::getNumberOfEdges, "returns the number of nodes")
            .def("getTimeInterval", &OrderedEdgeList<TemporalEdge>::getTimeInterval, "returns the time interval spanned by the temporal graph")
            .def("getEdges", &OrderedEdgeList<TemporalEdge>::getEdges)
            .def("getReverseNodeMap", &OrderedEdgeList<TemporalEdge>::getReverseNodeMap)
            .def("getNodeMap", &OrderedEdgeList<TemporalEdge>::getNodeMap)
            ;

    m.def("get_statistics",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&>(&get_statistics<TemporalEdge>));

}

} // tglib_python_binding