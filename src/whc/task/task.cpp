#include <dart/dynamics/BodyNode.hpp>

#include <whc/task/task.hpp>

namespace whc {
    namespace task {
        // AccelerationTask
        AccelerationTask::AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired)
            : _skeleton(skeleton), _body_name(body_name), _desired_accelerations(desired) {}

        std::pair<Eigen::MatrixXd, Eigen::VectorXd> AccelerationTask::get_costs()
        {
            _calculate_jacobian();
            // Returns A matrix with only the acceleration part
            // the QPSolver should fix/resize the matrices
            Eigen::MatrixXd A = _jacobian;
            Eigen::VectorXd b = _desired_accelerations - _jacobian_deriv * _dq;

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
        COMAccelerationTask::COMAccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired)
            : _skeleton(skeleton), _desired_accelerations(desired) {}

        std::pair<Eigen::MatrixXd, Eigen::VectorXd> COMAccelerationTask::get_costs()
        {
            _calculate_jacobian();
            // Returns A matrix with only the acceleration part
            // the QPSolver should fix/resize the matrices
            Eigen::MatrixXd A = _jacobian;
            Eigen::VectorXd b = _desired_accelerations - _jacobian_deriv * _dq;

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
        DirectTrackingTask::DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired)
            : _skeleton(skeleton), _desired_values(desired) {}

        std::pair<Eigen::MatrixXd, Eigen::VectorXd> DirectTrackingTask::get_costs()
        {
            // Assumes desired values to be same size of optimization variables
            Eigen::MatrixXd A = Eigen::MatrixXd::Identity(_desired_values.size(), _desired_values.size());
            Eigen::VectorXd b = _desired_values;

            return std::make_pair(A, b);
        }

        std::string DirectTrackingTask::get_type() const
        {
            return "direct";
        }

        // TauDiffTask
        TauDiffTask::TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau)
            : _skeleton(skeleton), _prev_tau(prev_tau) {}

        std::pair<Eigen::MatrixXd, Eigen::VectorXd> TauDiffTask::get_costs()
        {
            size_t dofs = _skeleton->getNumDofs();
            // Does not return anything for the contacts
            // the QPSolver should fix/resize the matrices
            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(2 * dofs, 2 * dofs);
            A.diagonal().tail(dofs) = Eigen::VectorXd::Ones(dofs);
            Eigen::VectorXd b = Eigen::VectorXd::Zero(2 * dofs);
            b.tail(dofs) = _prev_tau;

            return std::make_pair(A, b);
        }

        std::string TauDiffTask::get_type() const
        {
            return "tau_diff";
        }
    } // namespace task
} // namespace whc