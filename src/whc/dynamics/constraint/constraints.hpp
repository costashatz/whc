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
#ifndef WHC_DYNAMICS_CONSTRAINT_CONSTRAINTS_HPP
#define WHC_DYNAMICS_CONSTRAINT_CONSTRAINTS_HPP

#include <whc/abstract_constraint.hpp>
#include <whc/abstract_whc_solver.hpp>
#include <whc/utils/common.hpp>

namespace whc {
    namespace dyn {
        namespace constraint {
            class DynamicsConstraint : public AbstractConstraint {
            public:
                DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base = true);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                size_t N() const override;
                std::string get_type() const override;

            protected:
                bool _floating_base;
            };

            class ContactConstraint : public AbstractContactConstraint {
            public:
                ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const utils::Contact& contact);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                Eigen::MatrixXd get_jacobian() const override;
                Eigen::MatrixXd get_force_limits() const override;

                size_t N() const override;
                std::string get_type() const override;

                void set_contact(const utils::Contact& contact);

            protected:
                utils::Contact _contact;
            };

            class JointLimitsConstraint : public AbstractConstraint {
            public:
                JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                size_t N() const override;
                std::string get_type() const override;
            };
        } // namespace constraint
    } // namespace dyn
} // namespace whc

#endif