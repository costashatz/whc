#include <memory>

#include <qpOASES.hpp>

#include <whc/solver/qp_oases.hpp>

namespace whc {
    namespace solver {
        void QPOases::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            static std::unique_ptr<qpOASES::SQProblem> qp_solver = nullptr;
            assert(H.rows() == H.cols());
            assert(H.rows() == g.size());
            assert(lb.size() == ub.size() && ub.size() == g.size());
            assert(A.cols() == g.size());
            assert(A.rows() == lbA.size() && lbA.size() == ubA.size());

            size_t dim = g.size();
            size_t num_constraints = A.rows();
            bool first = false;
            if (!qp_solver) {
                qp_solver = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(dim, num_constraints));
                first = true;
            }

            // TO-DO: Set this options from outside
            auto options = qp_solver->getOptions();
            options.printLevel = qpOASES::PL_LOW;
            // options.enableFarBounds = qpOASES::BT_TRUE;
            // options.enableFlippingBounds = qpOASES::BT_TRUE;
            options.enableRamping = qpOASES::BT_FALSE;
            options.enableNZCTests = qpOASES::BT_FALSE;
            options.enableDriftCorrection = 0;
            options.terminationTolerance = 1e-6;
            options.boundTolerance = 1e-4;
            options.epsIterRef = 1e-6;

            qp_solver->setOptions(options);
            // options.print();
            int nWSR = 1000;
            double max_time = 0.005;

            // qpOASES uses row-major storing
            qpOASES::real_t* H_qp = new qpOASES::real_t[dim * dim];
            qpOASES::real_t* A_qp = new qpOASES::real_t[num_constraints * dim];
            qpOASES::real_t* g_qp = new qpOASES::real_t[dim];
            qpOASES::real_t* lb_qp = new qpOASES::real_t[dim];
            qpOASES::real_t* ub_qp = new qpOASES::real_t[dim];
            qpOASES::real_t* lbA_qp = new qpOASES::real_t[num_constraints];
            qpOASES::real_t* ubA_qp = new qpOASES::real_t[num_constraints];

            for (int i = 0; i < H.rows(); i++) {
                for (int j = 0; j < H.cols(); j++) {
                    H_qp[i * H.cols() + j] = H(i, j);
                }
            }

            for (int i = 0; i < A.rows(); i++) {
                for (int j = 0; j < A.cols(); j++) {
                    A_qp[i * A.cols() + j] = A(i, j);
                }
            }

            for (int i = 0; i < g.size(); i++) {
                g_qp[i] = g(i);
                lb_qp[i] = lb(i);
                ub_qp[i] = ub(i);
            }

            for (size_t i = 0; i < num_constraints; i++) {
                lbA_qp[i] = lbA(i);
                ubA_qp[i] = ubA(i);
            }

            qpOASES::SymDenseMat H_mat(H.rows(), H.cols(), H.cols(), H_qp);
            qpOASES::SparseMatrix A_mat(A.rows(), A.cols(), A.cols(), A_qp);

            if (first)
                qp_solver->init(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, nWSR);
            else
                qp_solver->hotstart(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, nWSR, &max_time);

            delete[] H_qp;
            delete[] A_qp;
            delete[] g_qp;
            delete[] lb_qp;
            delete[] ub_qp;
            delete[] lbA_qp;
            delete[] ubA_qp;

            _solution = Eigen::VectorXd(dim);
            qp_solver->getPrimalSolution(_solution.data());
        }

        Eigen::VectorXd QPOases::get_solution() const
        {
            return _solution;
        }
    } // namespace solver
} // namespace whc