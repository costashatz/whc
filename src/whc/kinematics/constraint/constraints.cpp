#include <dart/dynamics/BodyNode.hpp>

#include <whc/kinematics/constraint/constraints.hpp>

namespace whc {
    namespace kin {
        namespace constraint {
            JointLimitsConstraint::JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton) : AbstractConstraint(skeleton) {}

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> JointLimitsConstraint::data(AbstractWhcSolver* solver)
            {
                size_t dofs = _skeleton->getNumDofs();
                double dt = _skeleton->getTimeStep();
                Eigen::VectorXd q = _skeleton->getPositions();

                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dofs, solver->dim());
                A.diagonal().head(dofs) = Eigen::VectorXd::Constant(dofs, dt);

                Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, dofs);
                bounds.row(0) = _skeleton->getPositionLowerLimits().transpose();
                bounds.row(0) -= q.transpose();
                bounds.row(1) = _skeleton->getPositionUpperLimits().transpose();
                bounds.row(1) -= q.transpose();

                return std::make_pair(A, bounds);
            }

            size_t JointLimitsConstraint::N() const
            {
                return _skeleton->getNumDofs();
            }

            std::string JointLimitsConstraint::get_type() const
            {
                return "joint_limits";
            }
        } // namespace constraint
    } // namespace kin
} // namespace whc