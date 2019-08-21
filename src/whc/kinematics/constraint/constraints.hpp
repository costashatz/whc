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
#ifndef WHC_KINEMATICS_CONSTRAINT_CONSTRAINTS_HPP
#define WHC_KINEMATICS_CONSTRAINT_CONSTRAINTS_HPP

#include <whc/abstract_constraint.hpp>
#include <whc/abstract_whc_solver.hpp>
#include <whc/utils/common.hpp>

namespace whc {
    namespace kin {
        namespace constraint {
            class JointLimitsConstraint : public AbstractConstraint {
            public:
                JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                size_t N() const override;
                std::string get_type() const override;
            };
        } // namespace constraint
    } // namespace kin
} // namespace whc

#endif