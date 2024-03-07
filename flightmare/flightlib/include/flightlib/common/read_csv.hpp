#pragma once

/*
 * MAT-file diagnose program
 *
 * See the MATLAB API Guide for compiling information.
 *
 * Calling syntax:
 *
 *   matdgns <matfile>
 *
 * It will diagnose the MAT-file named <matfile>.
 *
 * This program demonstrates the use of the following functions:
 *
 *  matClose
 *  matGetDir
 *  matGetNextVariable
 *  matGetNextVariableInfo
 *  matOpen
 *
 * Copyright 1984-2003 The MathWorks, Inc.
 */

// All matlab files are included only for the liseagle model

#include <stdio.h>
#include <stdlib.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <unsupported/Eigen/CXX11/Tensor>
#include <vector>

#include "flightlib/common/types.hpp"
#include "flightlib/common/utils.hpp"

namespace flightlib {

bool readCsv(std::string path, Scalar &fill);
bool readCsv(std::string path, Ref<Matrix<Dynamic, Dynamic>> fill);

// Tensor needs to be fixed size otherwise it can't be initialized in header file.
// This leads to three identical functions (couldn't think of a better way)
bool readCsv(std::string path,
             Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 3, 3>> &fill);
bool readCsv(std::string path,
             Eigen::TensorFixedSize<Scalar, Eigen::Sizes<11, 2, 4>> &fill);
bool readCsv(std::string path,
             Eigen::TensorFixedSize<Scalar, Eigen::Sizes<3, 3, 3>> &fill);

Matrix<Dynamic, Dynamic> read(std::string path);

// int readMat(const char *file);

}  // namespace flightlib