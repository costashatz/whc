//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Author/Maintainer:  Konstantinos Chatzilygeroudis
//|    email:   costashatz@gmail.com
//|    website: lasa.epfl.ch
//|
//|    This file is part of whc.
//|
//|    whc is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    whc is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
#include <whc/utils/math.hpp>

#include <cmath>
#include <limits>

#include <dart/math/Geometry.hpp>

namespace whc {
    namespace utils {
        Eigen::Vector3d rotation_error(const Eigen::Matrix3d& R_desired, const Eigen::Matrix3d& R_current)
        {
            return dart::math::logMap(R_desired * R_current.transpose());
        }
    } // namespace utils
} // namespace whc