#include <memory>

#include <cppipm.h>

#include <whc/solver/cppipm.hpp>

namespace whc {
    namespace solver {
        void CPPIPM::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            size_t dim = g.size();
            size_t num_constraints = 2 * A.rows() + 2 * dim; // 2 * num of constraints + 2 * bounds

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
            std::unique_ptr<Algorithm> qp_solver = std::make_unique<cppipm>(H, A_qp, b_qp, g);
            static_cast<cppipm*>(qp_solver.get())->getParameters().set_verbose(0);
            qp_solver->solve();
            _solution = static_cast<cppipm*>(qp_solver.get())->getProblem().getOptx();
        }

        Eigen::VectorXd CPPIPM::get_solution() const
        {
            return _solution;
        }
    } // namespace solver
} // namespace whc