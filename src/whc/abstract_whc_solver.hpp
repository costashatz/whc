#ifndef WHC_WHC_SOLVER_HPP
#define WHC_WHC_SOLVER_HPP

#include <dart/dynamics/Skeleton.hpp>

#include <whc/abstract_constraint.hpp>
#include <whc/abstract_task.hpp>
#include <whc/qp_solver/abstract_qp.hpp>

namespace whc {
    class AbstractWhcSolver {
    public:
        AbstractWhcSolver();
        AbstractWhcSolver(const dart::dynamics::SkeletonPtr& skeleton);

        void set_skeleton(const dart::dynamics::SkeletonPtr& skeleton);

        qp_solver::AbstractQP* get_qp_solver() const;

        template <typename QPSolver, typename... Args>
        void set_qp_solver(Args... args)
        {
            _solver = std::unique_ptr<QPSolver>(new QPSolver(std::forward<Args>(args)...));
        }

        void clear_all();
        virtual void solve() = 0;

        void add_task(std::unique_ptr<AbstractTask> task);
        void add_constraint(std::unique_ptr<AbstractConstraint> constraint);

        size_t dim();
        std::vector<std::unique_ptr<AbstractTask>>& tasks();
        std::vector<std::unique_ptr<AbstractContactConstraint>>& contacts();
        std::vector<std::unique_ptr<AbstractConstraint>>& constraints();

        Eigen::VectorXd solution() const;

    protected:
        std::unique_ptr<qp_solver::AbstractQP> _solver = nullptr;
        dart::dynamics::SkeletonPtr _skeleton;
        std::vector<std::unique_ptr<AbstractTask>> _tasks;
        std::vector<std::unique_ptr<AbstractContactConstraint>> _contact_constraints;
        std::vector<std::unique_ptr<AbstractConstraint>> _constraints;

        // QP data
        size_t _dim, _num_constraints;

        // QP matrices
        Eigen::MatrixXd _H, _A;
        Eigen::VectorXd _g, _ub, _lb, _ubA, _lbA;
    };
} // namespace whc

#endif