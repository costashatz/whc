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
                Eigen::VectorXd _desired_velocities;

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

            protected:
                Eigen::VectorXd _desired_values;
            };

            class VelDiffTask : public AbstractTask {
            public:
                VelDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_vel, double weight = 1.)
                    : VelDiffTask(skeleton, prev_vel, Eigen::VectorXd::Constant(prev_vel.size(), weight)) {}
                VelDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_vel, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;

            protected:
                Eigen::VectorXd _prev_vel;
            };
        } // namespace task
    } // namespace kin
} // namespace whc

#endif