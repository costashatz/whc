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
            QPSolver(model::iCub* icub) : _icub(icub) {}

            void set_icub(model::iCub* icub)
            {
                _icub = icub;
            }

            void clear_all()
            {
                _tasks.clear();
            }

            void solve()
            {
                _setup_matrices();
                _solve();
            }

            void add_task(std::unique_ptr<task::AbstractTask> task)
            {
                _tasks.emplace_back(std::move(task));
            }

            void add_constraint(std::unique_ptr<constraint::AbstractConstraint> constraint)
            {
                if (constraint->get_type() != "contact") {
                    _constraints.emplace_back(std::move(constraint));
                }
                // TO-DO: warning message
            }

            void add_contact(const std::string& body_name, double mu)
            {
                // Add contact constraint
                _contact_constraints.emplace_back(constraint::create_constraint<constraint::ContactConstraint>(_icub->skeleton(), body_name, mu));
                // Add zero acceleration task
                _tasks.emplace_back(task::create_task<task::AccelerationTask>(_icub->skeleton(), body_name, Eigen::VectorXd::Zero(6)));
            }

            size_t dim() { return _dim; }
            std::vector<std::unique_ptr<task::AbstractTask>>& tasks() { return _tasks; }
            std::vector<std::unique_ptr<constraint::ContactConstraint>>& contacts() { return _contact_constraints; }
            std::vector<std::unique_ptr<constraint::AbstractConstraint>>& constraints() { return _constraints; }

        protected:
            std::unique_ptr<qpOASES::QProblem> _solver = nullptr;
            model::iCub* _icub;
            std::vector<std::unique_ptr<task::AbstractTask>> _tasks;
            std::vector<std::unique_ptr<constraint::ContactConstraint>> _contact_constraints;
            std::vector<std::unique_ptr<constraint::AbstractConstraint>> _constraints;

            // QP matrices
            size_t _dim;
            Eigen::MatrixXd _H, _A;
            Eigen::VectorXd _g, _ub, _lb, _ubA, _lbA;

            void _setup_matrices()
            {
                size_t dofs = _icub->skeleton()->getNumDofs();
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
                    A.block(index, 0, A_matrices[i].rows(), A_matrices[i].cols()) = A_matrices[i];
                    index += A_matrices[i].rows();

                    b.segment(b_index, b_vectors[i].size()) = b_vectors[i].transpose();
                    b_index += b_vectors[i].size();
                }

                _H = A.transpose() * A;
                _g = -A.transpose() * b;

                _lb = Eigen::VectorXd::Zero(_dim);
                _ub = Eigen::VectorXd::Zero(_dim);

                _lb.head(dofs) = _icub->robot()->skeleton()->getAccelerationLowerLimits();
                _ub.head(dofs) = _icub->robot()->skeleton()->getAccelerationUpperLimits();

                _lb.segment(dofs, dofs) = _icub->robot()->skeleton()->getForceLowerLimits();
                _ub.segment(dofs, dofs) = _icub->robot()->skeleton()->getForceUpperLimits();

                // _lb.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Zero(30);
                // _ub.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Zero(30);
                // _lb.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Constant(5 * 6, -std::numeric_limits<double>::max());
                // _ub.segment(2 * dofs, 5 * 6) = Eigen::VectorXd::Constant(5 * 6, std::numeric_limits<double>::max());
                // TO-DO: Get somehow the force limits

                size_t num_of_constraints = 0;
                for (size_t i = 0; i < _constraints.size(); i++)
                    num_of_constraints += _constraints[i]->N();
                for (size_t i = 0; i < _contact_constraints.size(); i++)
                    num_of_constraints += _contact_constraints[i]->N();

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
                    std::tie(mat, bounds) = _contact_constraints[i]->data(*this);
                    _A.block(c_index, 0, mat.rows(), mat.cols()) = mat;
                    _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                    _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                    c_index += mat.rows();
                }
            }

            void _solve()
            {
                size_t dofs = _icub->robot()->skeleton()->getNumDofs();

                if (!_solver)
                    _solver = std::unique_ptr<qpOASES::QProblem>(new qpOASES::QProblem(_dim, dofs));

                auto options = _solver->getOptions();
                options.printLevel = qpOASES::PL_LOW;
                _solver->setOptions(options);
                int nWSR = 200;
                // qpOASES uses row-major storing
                // check if values are passed correctly
                _solver->init(_H.transpose().data(), _g.data(), _A.transpose().data(), _lb.data(), _ub.data(), _lbA.data(), _ubA.data(), nWSR);

                Eigen::VectorXd x(_dim);
                _solver->getPrimalSolution(x.data());
                std::cout << x.transpose() << std::endl;
            }
        };
    } // namespace solver
} // namespace icub

#endif