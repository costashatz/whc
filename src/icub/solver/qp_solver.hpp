#ifndef ICUB_SOLVER_QP_SOLVER_HPP
#define ICUB_SOLVER_QP_SOLVER_HPP

#include <qpOASES.hpp>

#include <icub/constraint/constraint.hpp>
#include <icub/model/iCub.hpp>
#include <icub/task/task.hpp>

namespace icub {
    namespace solver {
        class QPSolver {
        public:
            QPSolver() {}
            QPSolver(std::shared_ptr<robot_dart::Robot> robot) : _robot(robot) {}

            void set_icub(std::shared_ptr<robot_dart::Robot> robot)
            {
                _robot = robot;
            }

            void clear_all()
            {
                _tasks.clear();
                _task_weights.clear();
                _contact_constraints.clear();
                _constraints.clear();
            }

            void solve()
            {
                _setup_matrices();
                _solve();
            }

            void add_task(std::unique_ptr<task::AbstractTask> task, double weight = 1.)
            {
                _tasks.emplace_back(std::move(task));
                _task_weights.push_back(weight);
            }

            void add_constraint(std::unique_ptr<constraint::AbstractConstraint> constraint)
            {
                if (constraint->get_type() != "contact") {
                    _constraints.emplace_back(std::move(constraint));
                }
                // TO-DO: warning message
            }

            template <typename... Args>
            void add_contact(double weight, const std::string& body_name, Args... args)
            {
                // Add contact constraint
                _contact_constraints.emplace_back(constraint::create_constraint<constraint::ContactConstraint>(_robot->skeleton(), body_name, std::forward<Args>(args)...));
                // Add zero acceleration task
                _tasks.emplace_back(task::create_task<task::AccelerationTask>(_robot->skeleton(), body_name, Eigen::VectorXd::Zero(6)));
                _task_weights.push_back(weight);
            }

            size_t dim() { return _dim; }
            std::vector<std::unique_ptr<task::AbstractTask>>& tasks() { return _tasks; }
            std::vector<std::unique_ptr<constraint::ContactConstraint>>& contacts() { return _contact_constraints; }
            std::vector<std::unique_ptr<constraint::AbstractConstraint>>& constraints() { return _constraints; }

            Eigen::VectorXd solution() const { return _solution; }

        protected:
            std::unique_ptr<qpOASES::SQProblem> _solver = nullptr;
            std::shared_ptr<robot_dart::Robot> _robot;
            std::vector<std::unique_ptr<task::AbstractTask>> _tasks;
            std::vector<double> _task_weights;
            std::vector<std::unique_ptr<constraint::ContactConstraint>> _contact_constraints;
            std::vector<std::unique_ptr<constraint::AbstractConstraint>> _constraints;

            // QP matrices
            size_t _dim, _num_constraints;
            Eigen::MatrixXd _H, _A;
            Eigen::VectorXd _g, _ub, _lb, _ubA, _lbA;
            // QP solution
            Eigen::VectorXd _solution;

            void _setup_matrices()
            {
                size_t dofs = _robot->skeleton()->getNumDofs();
                size_t size = 0;
                size_t contacts = _contact_constraints.size();
                size_t N = contacts * 6;

                std::vector<Eigen::MatrixXd> A_matrices(_tasks.size());
                std::vector<Eigen::VectorXd> b_vectors(_tasks.size());

                for (size_t i = 0; i < _tasks.size(); i++) {
                    std::tie(A_matrices[i], b_vectors[i]) = _tasks[i]->get_costs();
                    size += A_matrices[i].rows();
                }

                _dim = 2 * dofs + N;
                Eigen::MatrixXd A = Eigen::MatrixXd::Zero(size, _dim);
                Eigen::VectorXd b = Eigen::VectorXd::Zero(size);

                size_t index = 0;
                size_t b_index = 0;
                for (size_t i = 0; i < _tasks.size(); i++) {
                    A.block(index, 0, A_matrices[i].rows(), A_matrices[i].cols()) = _task_weights[i] * A_matrices[i];
                    index += A_matrices[i].rows();

                    b.segment(b_index, b_vectors[i].size()) = _task_weights[i] * b_vectors[i].transpose();
                    b_index += b_vectors[i].size();
                }

                _H = A.transpose() * A;
                _g = -A.transpose() * b;

                _lb = Eigen::VectorXd::Zero(_dim);
                _ub = Eigen::VectorXd::Zero(_dim);

                _lb.head(dofs) = _robot->skeleton()->getAccelerationLowerLimits();
                _ub.head(dofs) = _robot->skeleton()->getAccelerationUpperLimits();

                _lb.segment(dofs, dofs) = _robot->skeleton()->getForceLowerLimits();
                _ub.segment(dofs, dofs) = _robot->skeleton()->getForceUpperLimits();

                for (size_t i = 0; i < contacts; i++) {
                    Eigen::MatrixXd bounds = _contact_constraints[i]->get_force_limits();
                    _lb.segment(2 * dofs + i * 6, 6) = bounds.row(0);
                    _ub.segment(2 * dofs + i * 6, 6) = bounds.row(1);
                }

                size_t num_of_constraints = 0;
                for (size_t i = 0; i < _constraints.size(); i++)
                    num_of_constraints += _constraints[i]->N();
                for (size_t i = 0; i < _contact_constraints.size(); i++)
                    num_of_constraints += _contact_constraints[i]->N();
                _num_constraints = num_of_constraints;

                _A = Eigen::MatrixXd::Zero(num_of_constraints, _dim);
                _ubA = Eigen::VectorXd::Zero(num_of_constraints);
                _lbA = Eigen::VectorXd::Zero(num_of_constraints);

                size_t c_index = 0;
                for (size_t i = 0; i < _constraints.size(); i++) {
                    Eigen::MatrixXd mat, bounds;
                    std::tie(mat, bounds) = _constraints[i]->data(*this);
                    _A.block(c_index, 0, mat.rows(), mat.cols()) = mat;
                    _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                    _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                    c_index += mat.rows();
                }

                for (size_t i = 0; i < _contact_constraints.size(); i++) {
                    Eigen::MatrixXd mat, bounds;
                    std::tie(mat, bounds) = _contact_constraints[i]->data(*this, i);
                    _A.block(c_index, 0, mat.rows(), mat.cols()) = mat;
                    _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                    _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                    c_index += mat.rows();
                }
            }

            void _solve()
            {
                bool first = false;
                if (!_solver) {
                    _solver = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(_dim, _num_constraints));
                    first = true;
                }

                // TO-DO: Set this options from outside
                auto options = _solver->getOptions();
                options.printLevel = qpOASES::PL_LOW;
                // options.enableFarBounds = qpOASES::BT_TRUE;
                // options.enableFlippingBounds = qpOASES::BT_TRUE;
                options.enableRamping = qpOASES::BT_FALSE;
                options.enableNZCTests = qpOASES::BT_FALSE;
                options.enableDriftCorrection = 0;
                options.terminationTolerance = 1e-6;
                options.boundTolerance = 1e-4;
                options.epsIterRef = 1e-6;

                _solver->setOptions(options);
                // options.print();
                int nWSR = 1000;
                double max_time = 0.005;

                // qpOASES uses row-major storing
                qpOASES::real_t* H = new qpOASES::real_t[_dim * _dim];
                qpOASES::real_t* A = new qpOASES::real_t[_num_constraints * _dim];
                qpOASES::real_t* g = new qpOASES::real_t[_dim];
                qpOASES::real_t* lb = new qpOASES::real_t[_dim];
                qpOASES::real_t* ub = new qpOASES::real_t[_dim];
                qpOASES::real_t* lbA = new qpOASES::real_t[_num_constraints];
                qpOASES::real_t* ubA = new qpOASES::real_t[_num_constraints];

                for (int i = 0; i < _H.rows(); i++) {
                    for (int j = 0; j < _H.cols(); j++) {
                        H[i * _H.cols() + j] = _H(i, j);
                    }
                }

                for (int i = 0; i < _A.rows(); i++) {
                    for (int j = 0; j < _A.cols(); j++) {
                        A[i * _A.cols() + j] = _A(i, j);
                    }
                }

                for (int i = 0; i < _g.size(); i++) {
                    g[i] = _g(i);
                    lb[i] = _lb(i);
                    ub[i] = _ub(i);
                }

                for (size_t i = 0; i < _num_constraints; i++) {
                    lbA[i] = _lbA(i);
                    ubA[i] = _ubA(i);
                }

                qpOASES::SymDenseMat H_mat(_H.rows(), _H.cols(), _H.cols(), H);
                qpOASES::SparseMatrix A_mat(_A.rows(), _A.cols(), _A.cols(), A);

                if (first)
                    _solver->init(&H_mat, g, &A_mat, lb, ub, lbA, ubA, nWSR);
                else
                    _solver->hotstart(&H_mat, g, &A_mat, lb, ub, lbA, ubA, nWSR, &max_time);

                delete[] H;
                delete[] A;
                delete[] g;
                delete[] lb;
                delete[] ub;
                delete[] lbA;
                delete[] ubA;

                _solution = Eigen::VectorXd(_dim);
                _solver->getPrimalSolution(_solution.data());
                // std::cout << x.transpose() << std::endl;
                // std::cout << _solver->getObjVal() << std::endl;
                // std::cout << "acc: " << x.head(38).transpose() << std::endl;
                // std::cout << "tau: " << x.segment(38 + 6, 32).transpose() << std::endl;
                // std::cout << "F: " << x.tail(x.size() - (38 + 6 + 32)).transpose() << std::endl;
            }
        };
    } // namespace solver
} // namespace icub

#endif