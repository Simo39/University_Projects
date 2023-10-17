/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

#include <pybind11/pybind11.h>
#include "../../algorithms/TemporalReachability.h"

namespace tglib_python_binding {

using namespace tglib;


/**
 * @file TemporalReachability_binding.cpp
 * @brief This file provides the python binding code.
 *
 */

void bind_TemporalReachability(pybind11::module_ &m) {
    m.def("number_of_reachable_nodes",
          pybind11::overload_cast < OrderedEdgeList < TemporalEdge > const&, NodeId, TimeInterval >
                                                                                     (&number_of_reachable_nodes <
                                                                                      TemporalEdge > ));

}

}