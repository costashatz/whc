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
#ifndef WHC_KINEMATICS_TASK_TASKS_HPP
#define WHC_KINEMATICS_TASK_TASKS_HPP

#include <whc/abstract_task.hpp>

namespace whc {
    namespace kin {
        namespace task {
            class VelocityTask : public AbstractTask {
            public:
                VelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, double weight = 1.)
                    : VelocityTask(skeleton, body_name, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
                VelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;

            protected:
                std::string _body_name;

                Eigen::MatrixXd _jacobian;

                void _calculate_jacobian();
            };

            class COMVelocityTask : public AbstractTask {
            public:
                COMVelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, double weight = 1.)
                    : COMVelocityTask(skeleton, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
                COMVelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;

            protected:
                Eigen::MatrixXd _jacobian;

                void _calculate_jacobian();
            };

            class DirectTrackingTask : public AbstractTask {
            public:
                DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, double weight = 1.)
                    : DirectTrackingTask(skeleton, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
                DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;
            };

            class VelDiffTask : public AbstractTask {
            public:
                VelDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_vel, double weight = 1.)
                    : VelDiffTask(skeleton, prev_vel, Eigen::VectorXd::Constant(prev_vel.size(), weight)) {}
                VelDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_vel, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;
            };
        } // namespace task
    } // namespace kin
} // namespace whc

#endif