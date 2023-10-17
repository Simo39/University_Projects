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
#include "../../core/Transformations.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_Transformations(pybind11::module_ &m) {
    m.def("to_incident_lists", &to_incident_lists<TGNode, TemporalEdge>);
    m.def("to_trs_graph", &to_trs_graph<TemporalEdge>);
    m.def("to_directed_line_graph",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&>(&to_directed_line_graph<TemporalEdge>));
    m.def("to_aggregated_edge_list", to_aggregated_edge_list<TemporalEdge>);
    m.def("normalize", normalize<TemporalEdge>);
    m.def("scale_timestamps", scale_timestamps<TemporalEdge>);
    m.def("unit_transition_times", unit_transition_times<TemporalEdge>);
}

} // tglib_python_binding