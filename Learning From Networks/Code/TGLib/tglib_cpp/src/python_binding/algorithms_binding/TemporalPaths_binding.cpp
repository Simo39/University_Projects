/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file TemporalPaths_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include "../../algorithms/TemporalPaths.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_TemporalPaths(pybind11::module_ &m) {
    m.def("minimum_duration_path", &minimum_duration_path<TGNode, TemporalEdge>);
    m.def("earliest_arrival_path", &earliest_arrival_path<TGNode, TemporalEdge>);
    m.def("minimum_transition_time_path", &minimum_transition_time_path<TGNode, TemporalEdge>);
    m.def("minimum_hops_path", &minimum_hops_path<TGNode, TemporalEdge>);
}

}