/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file Closeness_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include "../../algorithms/TemporalCloseness.h"
#include "../python_binding.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_TemporalCloseness(pybind11::module_ &m) {
    m.def("temporal_closeness",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, TimeInterval, Distance_Type>(&temporal_closeness<TemporalEdge>));

    m.def("temporal_closeness",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, NodeId, Distance_Type>(&temporal_closeness<TemporalEdge>));

    m.def("temporal_closeness",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, TimeInterval, Distance_Type>(&temporal_closeness<TemporalEdge>));

    m.def("temporal_closeness",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, Distance_Type>(&temporal_closeness<TemporalEdge>));

    // todo
    m.def("run_temporal_closeness",
          pybind11::overload_cast<OrderedEdgeList<TemporalEdge> const&, Distance_Type>(&run_temporal_closeness<OrderedEdgeList<TemporalEdge>>));

}

}