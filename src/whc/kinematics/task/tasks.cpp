#include <dart/dynamics/BodyNode.hpp>

#include <whc/kinematics/task/tasks.hpp>

namespace whc {
    namespace kin {
        namespace task {
            // VelocityTask
            VelocityTask::VelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, desired, weights), _body_name(body_name) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> VelocityTask::get_costs()
            {
                _calculate_jacobian();
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = _desired.array() * _weights.array();

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

            // COMVelocityTask
            COMVelocityTask::COMVelocityTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, desired, weights) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> COMVelocityTask::get_costs()
            {
                _calculate_jacobian();
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = _desired.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string COMVelocityTask::get_type() const
            {
                return "com_velocity";
            }

            void COMVelocityTask::_calculate_jacobian()
            {
                _jacobian = _skeleton->getCOMJacobian();
            }

            // DirectTrackingTask
            DirectTrackingTask::DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, desired, weights) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> DirectTrackingTask::get_costs()
            {
                // Assumes desired values to be same size of optimization variables
                Eigen::MatrixXd A = Eigen::MatrixXd::Identity(_desired.size(), _desired.size());
                A.diagonal().array() *= _weights.array();
                Eigen::VectorXd b = _desired.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string DirectTrackingTask::get_type() const
            {
                return "direct";
            }

            // VelDiffTask
            VelDiffTask::VelDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_vel, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, prev_vel, weights) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> VelDiffTask::get_costs()
            {
                size_t dofs = _skeleton->getNumDofs();

                Eigen::MatrixXd A = Eigen::MatrixXd::Identity(dofs, dofs);
                A.diagonal().array() *= _weights.array();
                Eigen::VectorXd b = _desired.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string VelDiffTask::get_type() const
            {
                return "vel_diff";
            }
        } // namespace task
    } // namespace kin
} // namespace whc