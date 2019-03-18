#include <whc/abstract_whc_solver.hpp>

namespace whc {
    AbstractWhcSolver::AbstractWhcSolver() {}
    AbstractWhcSolver::AbstractWhcSolver(const dart::dynamics::SkeletonPtr& skeleton) : _skeleton(skeleton) {}

    void AbstractWhcSolver::set_skeleton(const dart::dynamics::SkeletonPtr& skeleton)
    {
        _skeleton = skeleton;
    }

    qp_solver::AbstractQP* AbstractWhcSolver::get_qp_solver() const
    {
        return _solver.get();
    }

    void AbstractWhcSolver::clear_all()
    {
        _tasks.clear();
        _contact_constraints.clear();
        _constraints.clear();
    }

    void AbstractWhcSolver::add_task(std::unique_ptr<AbstractTask> task)
    {
        _tasks.emplace_back(std::move(task));
    }

    void AbstractWhcSolver::add_constraint(std::unique_ptr<AbstractConstraint> constraint)
    {
        if (constraint->get_type() != "contact")
            _constraints.emplace_back(std::move(constraint));
        else
            _contact_constraints.emplace_back(std::unique_ptr<AbstractContactConstraint>{static_cast<AbstractContactConstraint*>(constraint.release())});
    }

    size_t AbstractWhcSolver::dim() { return _dim; }
    std::vector<std::unique_ptr<AbstractTask>>& AbstractWhcSolver::tasks() { return _tasks; }
    std::vector<std::unique_ptr<AbstractContactConstraint>>& AbstractWhcSolver::contacts() { return _contact_constraints; }
    std::vector<std::unique_ptr<AbstractConstraint>>& AbstractWhcSolver::constraints() { return _constraints; }

    Eigen::VectorXd AbstractWhcSolver::solution() const
    {
        if (_solver)
            return _solver->get_solution();
        return Eigen::VectorXd();
    }
} // namespace whc