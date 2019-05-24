#include <dart/dynamics/BodyNode.hpp>

#include <whc/dynamics/constraint/constraints.hpp>

namespace whc {
    namespace dyn {
        namespace constraint {
            DynamicsConstraint::DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base)
                : AbstractConstraint(skeleton), _floating_base(floating_base) {}

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DynamicsConstraint::data(AbstractWhcSolver* solver)
            {
                size_t dofs = _skeleton->getNumDofs();
                size_t contacts = solver->contacts().size();
                // Get mass matrix
                Eigen::MatrixXd M = _skeleton->getMassMatrix();
                // Get gravity/coriolis forces
                Eigen::VectorXd Cg = _skeleton->getCoriolisAndGravityForces();
                // Selection matrix
                Eigen::MatrixXd S = Eigen::MatrixXd::Identity(dofs, dofs);
                if (_floating_base)
                    S.diagonal().head(6) = Eigen::VectorXd::Zero(6);

                Eigen::MatrixXd A(dofs, solver->dim());

                A.block(0, 0, dofs, dofs) = M;
                A.block(0, dofs, dofs, dofs) = -S;
                size_t index = 2 * dofs;
                for (size_t i = 0; i < contacts; i++) {
                    Eigen::MatrixXd jac = solver->contacts()[i]->get_jacobian();
                    size_t r = jac.rows();
                    size_t c = jac.cols();
                    A.block(0, index, c, r) = -jac.transpose();
                    index += r;
                }

                Eigen::MatrixXd bounds(2, dofs);
                bounds.row(0) = -Cg.transpose();
                bounds.row(1) = -Cg.transpose();

                return std::make_pair(A, bounds);
            }

            size_t DynamicsConstraint::N() const
            {
                return _skeleton->getNumDofs();
            }

            std::string DynamicsConstraint::get_type() const
            {
                return "dynamics";
            }

            ContactConstraint::ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const utils::Contact& contact)
                : AbstractContactConstraint(skeleton, body_name), _contact(contact) {}

            // Friction cones from:
            //  M. Focchi, A. Del Prete, I. Havoutis, R. Featherstone, D. G.
            // Caldwell, and C. Semini, “High-slope terrain locomotion
            // for torque-controlled quadruped robots,” Autonomous Robots,
            // vol. 41, no. 1, pp. 259–272, 2017
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ContactConstraint::data(AbstractWhcSolver* solver)
            {
                double max = std::numeric_limits<double>::max();
                double min_f = _contact.nz.dot(_contact.min.tail(3));
                double max_f = _contact.nz.dot(_contact.max.tail(3));
                size_t num_constraints = 5;
                if (_contact.cop_constraint)
                    num_constraints += 6;

                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, 6);
                // Force
                A.block(0, 3, 1, 3) = -(_contact.mu_s * _contact.nz + _contact.nx).transpose();
                A.block(1, 3, 1, 3) = -(_contact.mu_s * _contact.nz + _contact.ny).transpose();
                A.block(2, 3, 1, 3) = (_contact.mu_s * _contact.nz + _contact.nx).transpose();
                A.block(3, 3, 1, 3) = (_contact.mu_s * _contact.nz + _contact.ny).transpose();
                A.block(4, 3, 1, 3) = _contact.nz.transpose();
                // CoP constraint
                // TO-DO: CHECK IF THIS IS CORRECT
                if (_contact.cop_constraint) {
                    double d_y_min = _contact.d_y_min;
                    double d_y_max = _contact.d_y_max;
                    double d_x_min = _contact.d_x_min;
                    double d_x_max = _contact.d_x_max;
                    Eigen::MatrixXd R = _skeleton->getBodyNode(_body_name)->getTransform().linear();
                    Eigen::VectorXd local_nx, local_ny, local_nz;
                    local_nx = R * _contact.nx;
                    local_ny = R * _contact.ny;
                    local_nz = R * _contact.nz;
                    // first constraint: 0 <= -T_y^b - d_x_min*F_z^b <= max
                    A.block(5, 0, 1, 3) << -R(1, 0), -R(1, 1), -R(1, 2);
                    A.block(5, 0, 1, 3).array() *= local_ny.transpose().array();
                    A.block(5, 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                    A.block(5, 3, 1, 3).array() *= -d_x_min * local_nz.transpose().array();
                    // second constraint: -max <= -T_y^b - d_x_max*F_z^b <= 0
                    A.block(6, 0, 1, 3) << -R(1, 0), -R(1, 1), -R(1, 2);
                    A.block(6, 0, 1, 3).array() *= local_ny.transpose().array();
                    A.block(6, 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                    A.block(6, 3, 1, 3).array() *= -d_x_max * local_nz.transpose().array();
                    // third constraint: 0 <= T_x^b - d_y_min*F_z^b <= max
                    A.block(7, 0, 1, 3) << R(0, 0), R(0, 1), R(0, 2);
                    A.block(7, 0, 1, 3).array() *= local_nx.transpose().array();
                    A.block(7, 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                    A.block(7, 3, 1, 3).array() *= -d_y_min * local_nz.transpose().array();
                    // fourth constraint: -max <= T_x^b - d_y_max*F_z^b <= 0
                    A.block(8, 0, 1, 3) << R(0, 0), R(0, 1), R(0, 2);
                    A.block(8, 0, 1, 3).array() *= local_nx.transpose().array();
                    A.block(8, 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                    A.block(8, 3, 1, 3).array() *= -d_y_max * local_nz.transpose().array();
                    // fifth constraint: 0 <= T*n - mu_k*F*n <= max
                    A.block(9, 0, 1, 3) = _contact.nz.transpose();
                    A.block(9, 3, 1, 3) = _contact.mu_k * _contact.nz.transpose();
                    // sixth constraint: -max <= T*n-mu_k*F*n <= 0
                    A.block(10, 0, 1, 3) = _contact.nz.transpose();
                    A.block(10, 3, 1, 3) = -_contact.mu_k * _contact.nz.transpose();

                    // // 1a
                    // // 0 <= T*t1-d_y_min*F*n <= max
                    // A.block(5, 0, 1, 3) = _contact.nx.transpose();
                    // A.block(5, 3, 1, 3) = -d_y_min * _contact.nz.transpose();

                    // // 1b
                    // // -max <= T*t1-d_y_max*F*n <= 0
                    // A.block(6, 0, 1, 3) = _contact.nx.transpose();
                    // A.block(6, 3, 1, 3) = -d_y_max * _contact.nz.transpose();

                    // // 2a
                    // // 0 <= T*t2-d_x_min*F*n <= max
                    // A.block(7, 0, 1, 3) = _contact.ny.transpose();
                    // A.block(7, 3, 1, 3) = -d_x_min * _contact.nz.transpose();

                    // // 2b
                    // // -max <= T*t2-d_x_max*F*n <= 0
                    // A.block(8, 0, 1, 3) = _contact.ny.transpose();
                    // A.block(8, 3, 1, 3) = -d_x_max * _contact.nz.transpose();

                    // // 3a
                    // // 0 <= T*n+mu_k*F*n <= max
                    // A.block(9, 0, 1, 3) = _contact.nz.transpose();
                    // A.block(9, 3, 1, 3) = _contact.mu_k * _contact.nz.transpose();

                    // // 3b
                    // // -max <= T*n-mu_k*F*n <= 0
                    // A.block(10, 0, 1, 3) = _contact.nz.transpose();
                    // A.block(10, 3, 1, 3) = -_contact.mu_k * _contact.nz.transpose();
                }
                // std::cout << A.block(0, 0, num_constraints, 6) << std::endl
                //           << std::endl;
                Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, num_constraints);
                bounds.row(0).head(5) << -max, -max, 0., 0., min_f;
                bounds.row(1).head(5) << 0., 0., max, max, max_f;
                if (_contact.cop_constraint) {
                    bounds.row(0).tail(6) << 0, -max, 0, -max, 0, -max;
                    bounds.row(1).tail(6) << max, 0, max, 0, max, 0;
                    // bounds.row(0).tail(6) << 0, -max, 0, -max, 0, -max;
                    // bounds.row(1).tail(6) << max, 0, max, 0, max, 0;
                }

                return std::make_pair(A, bounds);
            }

            Eigen::MatrixXd ContactConstraint::get_jacobian() const
            {
                auto bd = _skeleton->getBodyNode(_body_name);
                if (!bd)
                    return Eigen::MatrixXd();

                return _skeleton->getWorldJacobian(bd);
            }

            Eigen::MatrixXd ContactConstraint::get_force_limits() const
            {
                Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, 6);
                bounds.row(0) = _contact.min;
                bounds.row(1) = _contact.max;

                return bounds;
            }

            size_t ContactConstraint::N() const
            {
                return 5 + ((_contact.cop_constraint) ? 6 : 0);
            }

            std::string ContactConstraint::get_type() const
            {
                return "contact";
            }

            void ContactConstraint::set_contact(const utils::Contact& contact) { _contact = contact; }

            JointLimitsConstraint::JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton) : AbstractConstraint(skeleton) {}

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> JointLimitsConstraint::data(AbstractWhcSolver* solver)
            {
                size_t dofs = _skeleton->getNumDofs();
                double dt = _skeleton->getTimeStep();
                Eigen::VectorXd dq = _skeleton->getVelocities();
                Eigen::VectorXd q = _skeleton->getPositions();

                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dofs, solver->dim());
                A.diagonal().head(dofs) = Eigen::VectorXd::Constant(dofs, dt * dt);

                Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, dofs);
                bounds.row(0) = _skeleton->getPositionLowerLimits().transpose();
                bounds.row(0) -= (dq * dt + q).transpose();
                bounds.row(1) = _skeleton->getPositionUpperLimits().transpose();
                bounds.row(1) -= (dq * dt + q).transpose();

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
    } // namespace dyn
} // namespace whc