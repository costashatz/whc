#ifndef WHC_QP_SOLVER_QLD_HPP
#define WHC_QP_SOLVER_QLD_HPP

#include <whc/qp_solver/abstract_qp.hpp>

namespace whc {
    namespace qp_solver {
        class QLD : public AbstractQP {
        public:
            QLD();

            void solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA) override;
            Eigen::VectorXd get_solution() const override;

        protected:
            Eigen::VectorXd _solution;
        };
    } // namespace qp_solver
} // namespace whc

#endif