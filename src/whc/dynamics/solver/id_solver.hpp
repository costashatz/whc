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
#ifndef WHC_DYNAMICS_SOLVER_ID_SOLVER_HPP
#define WHC_DYNAMICS_SOLVER_ID_SOLVER_HPP

#include <dart/dynamics/Skeleton.hpp>

#include <whc/abstract_whc_solver.hpp>
#include <whc/dynamics/constraint/constraints.hpp>
#include <whc/dynamics/task/tasks.hpp>
#include <whc/utils/helpers.hpp>

namespace whc {
    namespace dyn {
        namespace solver {
            class IDSolver : public AbstractWhcSolver {
            public:
                IDSolver();
                IDSolver(const dart::dynamics::SkeletonPtr& skeleton);

                bool solve() override;

                template <typename... Args>
                void add_contact(double weight, const std::string& body_name, Args... args)
                {
                    add_contact(Eigen::VectorXd::Constant(6, weight), body_name, std::forward<Args>(args)...);
                }

                template <typename... Args>
                void add_contact(const Eigen::VectorXd& weights, const std::string& body_name, Args... args)
                {
                    // Add contact constraint
                    _contact_constraints.emplace_back(utils::make_unique<constraint::ContactConstraint>(_skeleton, body_name, std::forward<Args>(args)...));
                    // Add zero acceleration task
                    _tasks.emplace_back(utils::make_unique<task::AccelerationTask>(_skeleton, body_name, Eigen::VectorXd::Zero(6), weights));
                }

            protected:
                void _setup_matrices();
                bool _solve();
            };
        } // namespace solver
    } // namespace dyn
} // namespace whc

#endif