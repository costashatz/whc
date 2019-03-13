#ifndef WHC_SOLVER_WHC_SOLVER_HPP
#define WHC_SOLVER_WHC_SOLVER_HPP

#include <dart/dynamics/Skeleton.hpp>

#include <whc/constraint/constraint.hpp>
#include <whc/solver/abstract_qp.hpp>
#include <whc/task/task.hpp>
#include <whc/utils/helpers.hpp>

namespace robot_dart {
    class Robot;
}

namespace whc {
    namespace solver {
        class WhcSolver {
        public:
            WhcSolver();
            WhcSolver(std::shared_ptr<robot_dart::Robot> robot);

            void set_robot(const std::shared_ptr<robot_dart::Robot>& robot);

            AbstractQP* get_qp_solver() const;
            template <typename QPSolver, typename... Args>
            void set_qp_solver(Args... args)
            {
                _solver = std::unique_ptr<QPSolver>(new QPSolver(std::forward<Args>(args)...));
            }

            void clear_all();
            void solve();

            void add_task(std::unique_ptr<task::AbstractTask> task, double weight = 1.);
            void add_constraint(std::unique_ptr<constraint::AbstractConstraint> constraint);

            template <typename... Args>
            void add_contact(double weight, const std::string& body_name, Args... args)
            {
                // Add contact constraint
                _contact_constraints.emplace_back(utils::make_unique<constraint::ContactConstraint>(_robot->skeleton(), body_name, std::forward<Args>(args)...));
                // Add zero acceleration task
                _tasks.emplace_back(utils::make_unique<task::AccelerationTask>(_robot->skeleton(), body_name, Eigen::VectorXd::Zero(6)));
                _task_weights.push_back(weight);
            }

            size_t dim();
            std::vector<std::unique_ptr<task::AbstractTask>>& tasks();
            std::vector<std::unique_ptr<constraint::ContactConstraint>>& contacts();
            std::vector<std::unique_ptr<constraint::AbstractConstraint>>& constraints();

            Eigen::VectorXd solution() const;

        protected:
            std::unique_ptr<AbstractQP> _solver = nullptr;
            std::shared_ptr<robot_dart::Robot> _robot;
            std::vector<std::unique_ptr<task::AbstractTask>> _tasks;
            std::vector<double> _task_weights;
            std::vector<std::unique_ptr<constraint::ContactConstraint>> _contact_constraints;
            std::vector<std::unique_ptr<constraint::AbstractConstraint>> _constraints;

            // QP matrices
            size_t _dim, _num_constraints;
            Eigen::MatrixXd _H, _A;
            Eigen::VectorXd _g, _ub, _lb, _ubA, _lbA;

            void _setup_matrices();
            void _solve();
        };
    } // namespace solver
} // namespace whc

#endif