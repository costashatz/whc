#include <dart/dynamics/BodyNode.hpp>

#include <robot_dart/robot.hpp>

#include <whc/solver/whc_solver.hpp>

namespace whc {
    namespace solver {
        WhcSolver::WhcSolver() {}
        WhcSolver::WhcSolver(std::shared_ptr<robot_dart::Robot> robot) : _robot(robot) {}

        void WhcSolver::set_robot(const std::shared_ptr<robot_dart::Robot>& robot)
        {
            _robot = robot;
        }

        AbstractQP* WhcSolver::get_qp_solver() const
        {
            return _solver.get();
        }

        void WhcSolver::clear_all()
        {
            _tasks.clear();
            _task_weights.clear();
            _contact_constraints.clear();
            _constraints.clear();
        }

        void WhcSolver::solve()
        {
            _setup_matrices();
            _solve();
        }

        void WhcSolver::add_task(std::unique_ptr<task::AbstractTask> task, double weight)
        {
            _tasks.emplace_back(std::move(task));
            _task_weights.push_back(weight);
        }

        void WhcSolver::add_constraint(std::unique_ptr<constraint::AbstractConstraint> constraint)
        {
            if (constraint->get_type() != "contact") {
                _constraints.emplace_back(std::move(constraint));
            }
            // TO-DO: warning message
        }

        size_t WhcSolver::dim() { return _dim; }
        std::vector<std::unique_ptr<task::AbstractTask>>& WhcSolver::tasks() { return _tasks; }
        std::vector<std::unique_ptr<constraint::ContactConstraint>>& WhcSolver::contacts() { return _contact_constraints; }
        std::vector<std::unique_ptr<constraint::AbstractConstraint>>& WhcSolver::constraints() { return _constraints; }

        Eigen::VectorXd WhcSolver::solution() const
        {
            if (_solver)
                return _solver->get_solution();
            return Eigen::VectorXd();
        }

        void WhcSolver::_setup_matrices()
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
                size_t start_i = 2 * dofs + i * 6;

                Eigen::MatrixXd mat, bounds;
                std::tie(mat, bounds) = _contact_constraints[i]->data(*this);
                _A.block(c_index, start_i, mat.rows(), mat.cols()) = mat;

                _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                c_index += mat.rows();
            }
        }

        void WhcSolver::_solve()
        {
            if (_solver != nullptr)
                _solver->solve(_H, _g, _A, _lb, _ub, _lbA, _ubA);
            // else
            // ERROR
        }
    } // namespace solver
} // namespace whc