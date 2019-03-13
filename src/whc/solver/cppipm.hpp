#ifndef WHC_SOLVER_CPPIPM_HPP
#define WHC_SOLVER_CPPIPM_HPP

#include <whc/solver/abstract_qp.hpp>

namespace whc {
    namespace solver {
        class CPPIPM : public AbstractQP {
        public:
            void solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA) override;
            Eigen::VectorXd get_solution() const override;

        protected:
            Eigen::VectorXd _solution;
        };
    } // namespace solver
} // namespace whc

#endif