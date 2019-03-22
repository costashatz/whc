#include <dart/dynamics/BodyNode.hpp>

#include <whc/dynamics/task/tasks.hpp>

namespace whc {
    namespace dyn {
        namespace task {
            // AccelerationTask
            AccelerationTask::AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _body_name(body_name), _desired_accelerations(desired) { assert(weights.size() == desired.size()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> AccelerationTask::get_costs()
            {
                _calculate_jacobian();
                // Returns A matrix with only the acceleration part
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = (_desired_accelerations - _jacobian_deriv * _dq).array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string AccelerationTask::get_type() const
            {
                return "acceleration";
            }

            void AccelerationTask::_calculate_jacobian()
            {
                auto bd = _skeleton->getBodyNode(_body_name);
                if (!bd)
                    return;

                _jacobian = _skeleton->getWorldJacobian(bd);
                _jacobian_deriv = _skeleton->getJacobianSpatialDeriv(bd, dart::dynamics::Frame::World());

                _dq = _skeleton->getVelocities();
            }

            // COMAccelerationTask
            COMAccelerationTask::COMAccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _desired_accelerations(desired) { assert(weights.size() == desired.size()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> COMAccelerationTask::get_costs()
            {
                _calculate_jacobian();
                // Returns A matrix with only the acceleration part
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = (_desired_accelerations - _jacobian_deriv * _dq).array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string COMAccelerationTask::get_type() const
            {
                return "com_acceleration";
            }

            void COMAccelerationTask::_calculate_jacobian()
            {
                _jacobian = _skeleton->getCOMJacobian();
                _jacobian_deriv = _skeleton->getCOMJacobianSpatialDeriv();

                _dq = _skeleton->getVelocities();
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

            // TauDiffTask
            TauDiffTask::TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _prev_tau(prev_tau) { assert(weights.size() == prev_tau.size()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> TauDiffTask::get_costs()
            {
                size_t dofs = _skeleton->getNumDofs();
                // Does not return anything for the contacts
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * dofs, 2 * dofs);
                A.diagonal().tail(dofs) = Eigen::VectorXd::Ones(dofs);
                A.diagonal().array() *= _weights.array();
                Eigen::VectorXd b = Eigen::VectorXd::Zero(2 * dofs);
                b.tail(dofs) = _prev_tau;
                b.array() *= _weights.array();

                return std::make_pair(A, b);
            }

            std::string TauDiffTask::get_type() const
            {
                return "tau_diff";
            }

            // PostureTask
            PostureTask::PostureTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, weights), _desired_values(desired)
            {
                assert(weights.size() == desired.size());
                assert(skeleton->getNumDofs() == static_cast<size_t>(desired.size()));
            }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> PostureTask::get_costs()
            {
                size_t dofs = _skeleton->getNumDofs();
                double dt = _skeleton->getTimeStep();
                Eigen::VectorXd dq = _skeleton->getVelocities();
                Eigen::VectorXd q = _skeleton->getPositions();

                // Assumes desired values to be same size of dofs
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dofs, dofs);
                A.diagonal() = Eigen::VectorXd::Constant(dofs, dt * dt);
                A.diagonal().array() *= _weights.array();
                Eigen::VectorXd b = -(dq * dt + q) + _desired_values;
                b = b.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string PostureTask::get_type() const
            {
                return "posture";
            }
        } // namespace task
    } // namespace dyn
} // namespace whc