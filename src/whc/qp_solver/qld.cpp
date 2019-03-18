#include <memory>

#include <qld.h>

#include <whc/qp_solver/qld.hpp>

namespace whc {
    namespace qp_solver {
        QLD::QLD() {}

        void QLD::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            Eigen::MatrixXd Aeq, Aineq;
            Eigen::VectorXd beq, bineq;
            for (int i = 0; i < lbA.size(); i++) {
                if (lbA(i) == ubA(i)) {
                    Aeq.conservativeResize(Aeq.rows() + 1, A.cols());
                    Aeq.row(Aeq.rows() - 1) = A.row(i);

                    beq.conservativeResize(beq.size() + 1);
                    beq.tail(1)[0] = lbA(i);
                }
                else {
                    Aineq.conservativeResize(Aineq.rows() + 2, A.cols());
                    Aineq.row(Aineq.rows() - 2) = A.row(i);
                    Aineq.row(Aineq.rows() - 1) = -A.row(i);

                    bineq.conservativeResize(bineq.size() + 2);
                    bineq.tail(2)[0] = ubA(i);
                    bineq.tail(1)[0] = -lbA(i);
                }
            }

            Eigen::QLD solver(H.rows(), Aeq.rows(), Aineq.rows(), false);

            solver.solve(H, g,
                Aeq, beq,
                Aineq, bineq,
                lb, ub);

            _solution = solver.result();
        }

        Eigen::VectorXd QLD::get_solution() const
        {
            return _solution;
        }
    } // namespace qp_solver
} // namespace whc