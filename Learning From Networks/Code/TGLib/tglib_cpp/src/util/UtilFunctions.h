/* Copyright (C) 2022 Lutz Oettershagen - All Rights Reserved
 *
 * This file is part of TGLib which is released under MIT license.
 * See file LICENSE.md or go to https://gitlab.com/tgpublic/tglib
 * for full license details.
 */

/** @file UtilFunctions.h
 *  @brief Helpful utility functions
 */

#ifndef TGLIB_UTILFUNCTIONS_H
#define TGLIB_UTILFUNCTIONS_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <valarray>
#include <chrono>
#include <iostream>


namespace tglib {

/**
 * Computes mean and standard deviation.
 * @tparam T a numerical type
 * @param values vector of numerical values
 * @param mean the mean
 * @param stdev the standard deviation
 */
template<typename T>
void get_mean_std(std::vector<T> &values, double &mean, double &stdev) {
    double sum = std::accumulate(std::begin(values), std::end(values), 0.0);
    mean = sum / values.size();
    double accum = 0.0;
    std::for_each (std::begin(values), std::end(values), [&](const double d) {
        accum += (d - mean) * (d - mean);
    });
    stdev = sqrt(accum / (values.size()));
}

} // tglib

#endif //TGLIB_UTILFUNCTIONS_H
