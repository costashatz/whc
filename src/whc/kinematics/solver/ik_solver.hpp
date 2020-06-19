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
#ifndef WHC_KINEMATICS_SOLVER_IK_SOLVER_HPP
#define WHC_KINEMATICS_SOLVER_IK_SOLVER_HPP

#include <dart/dynamics/Skeleton.hpp>

#include <whc/abstract_whc_solver.hpp>
#include <whc/kinematics/constraint/constraints.hpp>
#include <whc/kinematics/task/tasks.hpp>
#include <whc/utils/helpers.hpp>

namespace whc {
    namespace kin {
        namespace solver {
            class IKSolver : public AbstractWhcSolver {
            public:
                IKSolver();
                IKSolver(const dart::dynamics::SkeletonPtr& skeleton);

                bool solve() override;

            protected:
                void _setup_matrices();
                bool _solve();
            };
        } // namespace solver
    } // namespace kin
} // namespace whc

#endif