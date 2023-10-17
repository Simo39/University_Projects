/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/**
 * @file Burstiness_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

#include <pybind11/pybind11.h>
#include "../../algorithms/Burstiness.h"

namespace tglib_python_binding {

using namespace tglib;

void bind_Burstiness(pybind11::module_ &m) {
    m.def("edge_burstiness",&edge_burstiness<TemporalEdge>);
    m.def("node_burstiness", &node_burstiness<TemporalEdge>);
}

}