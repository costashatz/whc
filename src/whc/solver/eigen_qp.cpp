#include <memory>

#include <external/eigenQP/eigen_qp.hpp>

#include <whc/solver/eigen_qp.hpp>

namespace whc {
    namespace solver {
        void EigenQP::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            using EigenQPSolver = ::EigenQP::QPIneqSolver<double, -1, -1>;

            size_t dim = g.size();
            size_t num_constraints = 2 * A.rows() + 2 * dim; // 2 * num of constraints + 2 * bounds
            EigenQPSolver qp_solver(dim, num_constraints);

            _solution.resize(dim);
            // EigenQP solves x^T*Qx + c^Tx
            // s.t. Ax <= b
            // H and g are already in format
            // we need to transform simple bounds to format
            Eigen::MatrixXd A_qp = Eigen::MatrixXd::Zero(num_constraints, dim);
            Eigen::VectorXd b_qp = Eigen::VectorXd::Zero(num_constraints);
            for (size_t i = 0; i < dim; i++) {
                // upper bounds
                A_qp(i, i) = 1.;
                b_qp(i) = ub(i);
                // lower bounds
                A_qp(2 * i, i) = -1.;
                b_qp(2 * i) = -lb(i);
            }
            // upper bounds
            A_qp.block(2 * dim, 0, A.rows(), A.cols()) = A;
            b_qp.segment(2 * dim, ubA.size()) = ubA.transpose();
            // lower bounds
            A_qp.block(2 * dim + A.rows(), 0, A.rows(), A.cols()) = -A;
            b_qp.segment(2 * dim + ubA.size(), lbA.size()) = -lbA.transpose();

            // solve the QP
            qp_solver.solve(H, g, A_qp, b_qp, _solution);
        }

        Eigen::VectorXd EigenQP::get_solution() const
        {
            return _solution;
        }
    } // namespace solver
} // namespace whc