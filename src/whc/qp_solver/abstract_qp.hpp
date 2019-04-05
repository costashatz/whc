#ifndef WHC_QP_SOLVER_ABSTRACT_QP_HPP
#define WHC_QP_SOLVER_ABSTRACT_QP_HPP

#include <Eigen/Core>

namespace whc {
    namespace qp_solver {
        class AbstractQP {
        public:
            virtual void solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA) = 0;
            virtual Eigen::VectorXd get_solution() const = 0;
            virtual ~AbstractQP() {}
        };
    } // namespace qp_solver
} // namespace whc

#endif