#include <icub/constraint/constraint.hpp>
#include <icub/solver/qp_solver.hpp>

namespace icub {
    namespace constraint {
        DynamicsConstraint::DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base)
            : _skeleton(skeleton), _floating_base(floating_base) {}

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> DynamicsConstraint::data(solver::QPSolver& solver)
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
            // TO-DO: Check this sign
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

        ContactConstraint::ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, double mu)
            : _skeleton(skeleton), _body_name(body_name), _mu(mu) {}

        std::pair<Eigen::MatrixXd, Eigen::MatrixXd> ContactConstraint::data(solver::QPSolver& solver)
        {
            // size_t dofs = _skeleton->getNumDofs();
            size_t dim = solver.dim();

            Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, dim);
            // Eigen::MatrixXd bounds = Eigen::MatrixXd::Ones(2, 6);
            // bounds.row(0) = -bounds.row(0);
            Eigen::MatrixXd bounds = Eigen::MatrixXd::Zero(2, 6);

            // TO-DO: Fix this

            return std::make_pair(A, bounds);
        }

        Eigen::MatrixXd ContactConstraint::get_jacobian() const
        {
            auto bd = _skeleton->getBodyNode(_body_name);
            if (!bd)
                return Eigen::MatrixXd();

            return _skeleton->getWorldJacobian(bd);
        }

        size_t ContactConstraint::N() const
        {
            return 6;
        }

        std::string ContactConstraint::get_type() const
        {
            return "contact";
        }
    } // namespace constraint
} // namespace icub