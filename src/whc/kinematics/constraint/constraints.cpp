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
#include <dart/dynamics/BodyNode.hpp>

#include <whc/kinematics/constraint/constraints.hpp>

namespace whc {
    namespace kin {
        namespace constraint {
            JointLimitsConstraint::JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton) : AbstractConstraint(skeleton) {}

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> JointLimitsConstraint::data(AbstractWhcSolver* solver)
            {
                size_t dofs = _skeleton->getNumDofs();
                double dt = _skeleton->getTimeStep();
                Eigen::VectorXd q = _skeleton->getPositions();

                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dofs, solver->dim());
                A.diagonal().head(dofs) = Eigen::VectorXd::Constant(dofs, dt);

                Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, dofs);
                bounds.row(0) = _skeleton->getPositionLowerLimits().transpose();
                bounds.row(0) -= q.transpose();
                bounds.row(1) = _skeleton->getPositionUpperLimits().transpose();
                bounds.row(1) -= q.transpose();

                return std::make_pair(A, bounds);
            }

            size_t JointLimitsConstraint::N() const
            {
                return _skeleton->getNumDofs();
            }

            std::string JointLimitsConstraint::get_type() const
            {
                return "joint_limits";
            }
        } // namespace constraint
    } // namespace kin
} // namespace whc