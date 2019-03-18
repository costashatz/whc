#include <whc/kinematics/solver/ik_solver.hpp>

namespace whc {
    namespace kin {
        namespace solver {
            IKSolver::IKSolver() : AbstractWhcSolver() {}
            IKSolver::IKSolver(const dart::dynamics::SkeletonPtr& skeleton) : AbstractWhcSolver(skeleton) {}

            void IKSolver::solve()
            {
                _setup_matrices();
                _solve();
            }

            void IKSolver::_setup_matrices()
            {
                // TO-DO: Add contacts?
                size_t dofs = _skeleton->getNumDofs();
                size_t size = 0;

                std::vector<Eigen::MatrixXd> A_matrices(_tasks.size());
                std::vector<Eigen::VectorXd> b_vectors(_tasks.size());

                for (size_t i = 0; i < _tasks.size(); i++) {
                    std::tie(A_matrices[i], b_vectors[i]) = _tasks[i]->get_costs();
                    size += A_matrices[i].rows();
                }

                _dim = dofs;
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size, _dim);
                Eigen::VectorXd b = Eigen::VectorXd::Zero(size);

                size_t index = 0;
                size_t b_index = 0;
                for (size_t i = 0; i < _tasks.size(); i++) {
                    A.block(index, 0, A_matrices[i].rows(), A_matrices[i].cols()) = A_matrices[i];
                    index += A_matrices[i].rows();

                    b.segment(b_index, b_vectors[i].size()) = b_vectors[i].transpose();
                    b_index += b_vectors[i].size();
                }

                _H = A.transpose() * A;
                _g = -A.transpose() * b;

                _lb = _skeleton->getVelocityLowerLimits();
                _ub = _skeleton->getVelocityUpperLimits();

                _num_constraints = 0;
                for (size_t i = 0; i < _constraints.size(); i++)
                    _num_constraints += _constraints[i]->N();

                _A = Eigen::MatrixXd::Zero(_num_constraints, _dim);
                _ubA = Eigen::VectorXd::Zero(_num_constraints);
                _lbA = Eigen::VectorXd::Zero(_num_constraints);

                size_t c_index = 0;
                for (size_t i = 0; i < _constraints.size(); i++) {
                    Eigen::MatrixXd mat, bounds;
                    std::tie(mat, bounds) = _constraints[i]->data(this);
                    _A.block(c_index, 0, mat.rows(), mat.cols()) = mat;
                    _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                    _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                    c_index += mat.rows();
                }
            }

            void IKSolver::_solve()
            {
                if (_solver != nullptr)
                    _solver->solve(_H, _g, _A, _lb, _ub, _lbA, _ubA);
                // else
                // ERROR
            }
        } // namespace solver
    } // namespace kin
} // namespace whc