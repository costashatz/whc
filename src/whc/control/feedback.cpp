//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Author/Maintainer:  Konstantinos Chatzilygeroudis
//|    email:   konstantinos.chatzilygeroudis@epfl.ch
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
#include <whc/control/feedback.hpp>
#include <whc/utils/math.hpp>

namespace whc {
    namespace control {
        Eigen::VectorXd feedback(const utils::Frame& state, const utils::ControlFrame& desired, const PDGains& gains)
        {
            Eigen::VectorXd pos_error = desired.pose - state.pose;
            pos_error.head(3) = utils::rotation_error(dart::math::expMapRot(desired.pose.head(3)), dart::math::expMapRot(state.pose.head(3)));

            return desired.acc + gains.kd.cwiseProduct(desired.vel - state.vel) + gains.kp.cwiseProduct(pos_error);
        }
    } // namespace control
} // namespace whc