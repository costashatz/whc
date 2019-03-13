#include <memory>

#include <external/eigenQP/eigen_qp.hpp>

#include <whc/solver/eigen_qp.hpp>

namespace whc {
    namespace solver {
        void EigenQP::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            using EigenQPSolver = ::EigenQP::QPIneqSolver<double, -1, -1>;

            size_t dim = g.size();
            size_t num_constraints = A.rows();
            EigenQPSolver qp_solver(dim, num_constraints);

            _solution.resize(dim);
            // qp_solver.solve(H, g, A, _solution);
        }

        Eigen::VectorXd EigenQP::get_solution() const
        {
            return _solution;
        }
    } // namespace solver
} // namespace whc