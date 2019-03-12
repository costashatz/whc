#include <icub/constraint/constraint.hpp>
#include <icub/solver/qp_solver.hpp>

namespace icub {
    namespace constraint {
        DynamicsConstraint::DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base)
            : _skeleton(skeleton), _floating_base(floating_base) {}

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DynamicsConstraint::data(solver::QPSolver& solver, size_t)
        {
            size_t dofs = _skeleton->getNumDofs();
            size_t contacts = solver.contacts().size();
            // Get mass matrix
            Eigen::MatrixXd M = _skeleton->getMassMatrix();
            // Get gravity/coriolis forces
            Eigen::VectorXd Cg = _skeleton->getCoriolisAndGravityForces();
            // Selection matrix
            Eigen::MatrixXd S = Eigen::MatrixXd::Identity(dofs, dofs);
            if (_floating_base)
                S.diagonal().head(6) = Eigen::VectorXd::Zero(6);

            Eigen::MatrixXd A(dofs, solver.dim());

            A.block(0, 0, dofs, dofs) = M;
            A.block(0, dofs, dofs, dofs) = -S;
            size_t index = 2 * dofs;
            for (size_t i = 0; i < contacts; i++) {
                Eigen::MatrixXd jac = solver.contacts()[i]->get_jacobian();
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

        ContactConstraint::ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Contact& contact)
            : _skeleton(skeleton), _body_name(body_name), _contact(contact) {}

        // Friction cones from:
        //  M. Focchi, A. Del Prete, I. Havoutis, R. Featherstone, D. G.
        // Caldwell, and C. Semini, “High-slope terrain locomotion
        // for torque-controlled quadruped robots,” Autonomous Robots,
        // vol. 41, no. 1, pp. 259–272, 2017
        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ContactConstraint::data(solver::QPSolver& solver, size_t index)
        {
            size_t dofs = _skeleton->getNumDofs();
            size_t dim = solver.dim();
            size_t start_i = 2 * dofs + index * 6;
            double max = std::numeric_limits<double>::max();
            double min_f = _contact.min_force;
            double max_f = _contact.max_force;
            size_t num_constraints = 5;
            if (_contact.calculate_torque)
                num_constraints += 6;

            Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_constraints, dim);
            // Force
            A.block(0, start_i + 3, 1, 3) = -(_contact.mu * _contact.normal + _contact.t1).transpose();
            A.block(1, start_i + 3, 1, 3) = -(_contact.mu * _contact.normal + _contact.t2).transpose();
            A.block(2, start_i + 3, 1, 3) = (_contact.mu * _contact.normal + _contact.t1).transpose();
            A.block(3, start_i + 3, 1, 3) = (_contact.mu * _contact.normal + _contact.t2).transpose();
            A.block(4, start_i + 3, 1, 3) = _contact.normal.transpose();
            // Torque
            if (_contact.calculate_torque) {
                double d_y_min = _contact.d_y_min;
                double d_y_max = _contact.d_y_max;
                double d_x_min = _contact.d_x_min;
                double d_x_max = _contact.d_x_max;
                // Eigen::MatrixXd R = _skeleton->getBodyNode(_body_name)->getTransform().linear();
                // // first constraint: 0 <= -T_y^b - d_x_min*F_z^b <= max
                // A.block(5, start_i, 1, 3) << -R(1, 0), -R(1, 1), -R(1, 2);
                // // A.block(5, start_i, 1, 3).array() *= _contact.normal[1];
                // A.block(5, start_i + 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                // A.block(5, start_i + 3, 1, 3).array() *= -d_x_min; // * _contact.normal[2];
                // // second constraint: -max <= -T_y^b - d_x_max*F_z^b <= 0
                // A.block(6, start_i, 1, 3) << -R(1, 0), -R(1, 1), -R(1, 2);
                // // A.block(6, start_i, 1, 3).array() *= _contact.normal[1];
                // A.block(6, start_i + 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                // A.block(6, start_i + 3, 1, 3).array() *= -d_x_max; // * _contact.normal[2];
                // // third constraint: 0 <= T_x^b - d_y_min*F_z^b <= max
                // A.block(7, start_i, 1, 3) << R(0, 0), R(0, 1), R(0, 2);
                // // A.block(7, start_i, 1, 3).array() *= _contact.normal[0];
                // A.block(7, start_i + 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                // A.block(7, start_i + 3, 1, 3).array() *= -d_y_min; // * _contact.normal[2];
                // // fourth constraint: -max <= T_x^b - d_y_max*F_z^b <= 0
                // A.block(8, start_i, 1, 3) << R(0, 0), R(0, 1), R(0, 2);
                // // A.block(8, start_i, 1, 3).array() *= _contact.normal[0];
                // A.block(8, start_i + 3, 1, 3) << R(2, 0), R(2, 1), R(2, 2);
                // A.block(8, start_i + 3, 1, 3).array() *= -d_y_max; // * _contact.normal[2];

                // 1a
                // 0 <= T*t1-d_x_min*F*n <= max
                A.block(5, start_i, 1, 3) = _contact.t1.transpose();
                A.block(5, start_i + 3, 1, 3) = -d_y_min * _contact.normal.transpose();

                // 1b
                // -max <= T*t1-d_x_max*F*n <= 0
                A.block(6, start_i, 1, 3) = _contact.t1.transpose();
                A.block(6, start_i + 3, 1, 3) = -d_y_max * _contact.normal.transpose();

                // 2a
                // 0 <= T*t2-d_y_min*F*n <= max
                A.block(7, start_i, 1, 3) = _contact.t2.transpose();
                A.block(7, start_i + 3, 1, 3) = -d_x_min * _contact.normal.transpose();

                // 2b
                // -max <= T*t2-d_y_max*F*n <= 0
                A.block(8, start_i, 1, 3) = _contact.t2.transpose();
                A.block(8, start_i + 3, 1, 3) = -d_x_max * _contact.normal.transpose();

                // 3a
                // 0 <= T*n+muR*F*n <= max
                A.block(9, start_i, 1, 3) = _contact.normal.transpose();
                A.block(9, start_i + 3, 1, 3) = _contact.muR * _contact.normal.transpose();

                // 3b
                // -max <= T*n-muR*F*n <= 0
                A.block(10, start_i, 1, 3) = _contact.normal.transpose();
                A.block(10, start_i + 3, 1, 3) = -_contact.muR * _contact.normal.transpose();
            }
            // std::cout << A.block(0, start_i, num_constraints, 6) << std::endl
            //           << std::endl;
            Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, num_constraints);
            bounds.row(0).head(5) << -max, -max, 0., 0., min_f;
            bounds.row(1).head(5) << 0., 0., max, max, max_f;
            if (_contact.calculate_torque) {
                // bounds.row(0).tail(4) << 0, -max, 0, -max;
                // bounds.row(1).tail(4) << max, 0, max, 0;
                bounds.row(0).tail(6) << 0, -max, 0, -max, 0, -max;
                bounds.row(1).tail(6) << max, 0, max, 0, max, 0;
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
            return 5 + ((_contact.calculate_torque) ? 6 : 0);
        }

        std::string ContactConstraint::get_type() const
        {
            return "contact";
        }
    } // namespace constraint
} // namespace icub