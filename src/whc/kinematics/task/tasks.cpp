#include <dart/dynamics/BodyNode.hpp>

#include <whc/kinematics/task/tasks.hpp>

namespace whc {
    namespace kin {
        namespace task {
            // VelocityTask
            VelocityTask::VelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _body_name(body_name), _desired_velocities(desired) { assert(weights.size() == desired.size()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> VelocityTask::get_costs()
            {
                _calculate_jacobian();
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = _desired_velocities.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string VelocityTask::get_type() const
            {
                return "velocity";
            }

            void VelocityTask::_calculate_jacobian()
            {
                auto bd = _skeleton->getBodyNode(_body_name);
                if (!bd)
                    return;

                _jacobian = _skeleton->getWorldJacobian(bd);
            }

            // DirectTrackingTask
            DirectTrackingTask::DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _desired_values(desired) { assert(weights.size() == desired.size()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> DirectTrackingTask::get_costs()
            {
                // Assumes desired values to be same size of optimization variables
                Eigen::MatrixXd A = Eigen::MatrixXd::Identity(_desired_values.size(), _desired_values.size());
                A.diagonal().array() *= _weights.array();
                Eigen::VectorXd b = _desired_values.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string DirectTrackingTask::get_type() const
            {
                return "direct";
            }

            // VelDiffTask
            VelDiffTask::VelDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_vel, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _prev_vel(prev_vel) { assert(weights.size() == prev_vel.size()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> VelDiffTask::get_costs()
            {
                size_t dofs = _skeleton->getNumDofs();

                Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dofs, dofs);
                A.diagonal().array() *= _weights.array();
                Eigen::VectorXd b = _prev_vel.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string VelDiffTask::get_type() const
            {
                return "vel_diff";
            }
        } // namespace task
    } // namespace dyn
} // namespace whc