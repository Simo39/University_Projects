/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file Distances_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include "../../algorithms/TemporalDistances.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_TemporalDistances(pybind11::module_ &m) {
    m.def("minimum_durations",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, TimeInterval>(&minimum_durations<TemporalEdge>));

    m.def("minimum_durations",
          pybind11::overload_cast<IncidentLists<TGNode, TemporalEdge> const&, NodeId, TimeInterval>(&minimum_durations<TGNode, TemporalEdge>));




    m.def("earliest_arrival_times",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, TimeInterval>(&earliest_arrival_times<TemporalEdge>));

    m.def("earliest_arrival_times",
          pybind11::overload_cast<IncidentLists<TGNode, TemporalEdge> const&, NodeId, TimeInterval>(&earliest_arrival_times<TGNode, TemporalEdge>));


    m.def("latest_departure_times",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, TimeInterval>(&latest_departure_times<TemporalEdge>));


    m.def("minimum_hops",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, TimeInterval>(&minimum_hops<TemporalEdge>));

    m.def("minimum_hops",
          pybind11::overload_cast<IncidentLists<TGNode, TemporalEdge> const&, NodeId, TimeInterval>(&minimum_hops<TGNode, TemporalEdge>));


    m.def("minimum_transition_times",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, TimeInterval>(&minimum_transition_times<TemporalEdge>));

    m.def("minimum_transition_times",
          pybind11::overload_cast<IncidentLists<TGNode, TemporalEdge> const&, NodeId, TimeInterval>(&minimum_transition_times<TGNode, TemporalEdge>));

}

}