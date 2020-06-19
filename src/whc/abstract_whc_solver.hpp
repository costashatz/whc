//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Author/Maintainer:  Konstantinos Chatzilygeroudis
//|    email:   costashatz@gmail.com
//|    website: lasa.epfl.ch
//|
//|    This file is part of whc.
//|
//|    whc is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    whc is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
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
        virtual ~AbstractWhcSolver(){}
        
        void set_skeleton(const dart::dynamics::SkeletonPtr& skeleton);

        qp_solver::AbstractQP* get_qp_solver() const;

        template <typename QPSolver, typename... Args>
        void set_qp_solver(Args... args)
        {
            _solver = std::unique_ptr<QPSolver>(new QPSolver(std::forward<Args>(args)...));
        }

        void clear_all();
        virtual bool solve() = 0;

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