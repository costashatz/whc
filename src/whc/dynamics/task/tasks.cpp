#include <dart/dynamics/BodyNode.hpp>

#include <whc/dynamics/task/tasks.hpp>

namespace whc {
    namespace dyn {
        namespace task {
            // AccelerationTask
            AccelerationTask::AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, desired, weights), _body_name(body_name) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> AccelerationTask::get_costs()
            {
                _calculate_jacobian();
                // Returns A matrix with only the acceleration part
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = (_desired - _jacobian_deriv * _dq).array() * _weights.array();

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
                : AbstractTask(skeleton, desired, weights) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> COMAccelerationTask::get_costs()
            {
                _calculate_jacobian();
                // Returns A matrix with only the acceleration part
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = _jacobian;
                for (int i = 0; i < A.rows(); i++)
                    A.row(i).array() *= _weights[i];
                Eigen::VectorXd b = (_desired - _jacobian_deriv * _dq).array() * _weights.array();

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

            // TauDiffTask
            TauDiffTask::TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau, const Eigen::VectorXd& weights)
                : AbstractTask(skeleton, prev_tau, weights) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> TauDiffTask::get_costs()
            {
                size_t dofs = _skeleton->getNumDofs();
                // Does not return anything for the contacts
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * dofs, 2 * dofs);
                A.diagonal().tail(dofs) = _weights;
                Eigen::VectorXd b = Eigen::VectorXd::Zero(2 * dofs);
                b.tail(dofs) = _desired.array() * _weights.array();

                return std::make_pair(A, b);
            }

            std::string TauDiffTask::get_type() const
            {
                return "tau_diff";
            }

            // PostureTask
            PostureTask::PostureTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights, bool floating_base)
                : AbstractTask(skeleton, desired, weights), _floating_base(floating_base) { assert(check_consistency()); }

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> PostureTask::get_costs()
            {
                size_t dofs = _skeleton->getNumDofs();
                size_t length = dofs;
                if (_floating_base)
                    length -= 6;
                double dt = _skeleton->getTimeStep();
                Eigen::VectorXd dq = _skeleton->getVelocities().tail(length);
                Eigen::VectorXd q = _skeleton->getPositions().tail(length);

                // Assumes desired values to be same size of dofs
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dofs, dofs);
                A.diagonal().tail(length) = Eigen::VectorXd::Constant(length, dt * dt);
                A.diagonal().tail(length).array() *= _weights.array();
                Eigen::VectorXd b = Eigen::VectorXd::Zero(dofs);
                b.tail(length) = -(dq * dt + q) + _desired;
                b.tail(length).array() *= _weights.array();

                return std::make_pair(A, b);
            }

            std::string PostureTask::get_type() const
            {
                return "posture";
            }

            bool PostureTask::check_consistency() const
            {
                if (!((_weights.size() == _desired.size()) || _weights.size() == 0 || _desired.size() == 0))
                    return false;
                size_t dofs = _skeleton->getNumDofs();
                if (_floating_base)
                    dofs -= 6;
                if (dofs != static_cast<size_t>(_desired.size()))
                    return false;
                return true;
            }
        } // namespace task
    } // namespace dyn
} // namespace whc