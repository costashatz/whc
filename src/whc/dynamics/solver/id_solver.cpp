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
#include <whc/dynamics/solver/id_solver.hpp>

namespace whc {
    namespace dyn {
        namespace solver {
            IDSolver::IDSolver() : AbstractWhcSolver() {}
            IDSolver::IDSolver(const dart::dynamics::SkeletonPtr& skeleton) : AbstractWhcSolver(skeleton) {}

            bool IDSolver::solve()
            {
                _setup_matrices();
                return _solve();
            }

            void IDSolver::_setup_matrices()
            {
                size_t dofs = _skeleton->getNumDofs();
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

                _lb.head(dofs) = _skeleton->getAccelerationLowerLimits();
                _ub.head(dofs) = _skeleton->getAccelerationUpperLimits();

                _lb.segment(dofs, dofs) = _skeleton->getForceLowerLimits();
                _ub.segment(dofs, dofs) = _skeleton->getForceUpperLimits();

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
                    std::tie(mat, bounds) = _constraints[i]->data(this);
                    _A.block(c_index, 0, mat.rows(), mat.cols()) = mat;
                    _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                    _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                    c_index += mat.rows();
                }

                for (size_t i = 0; i < _contact_constraints.size(); i++) {
                    size_t start_i = 2 * dofs + i * 6;

                    Eigen::MatrixXd mat, bounds;
                    std::tie(mat, bounds) = _contact_constraints[i]->data(this);
                    _A.block(c_index, start_i, mat.rows(), mat.cols()) = mat;

                    _lbA.segment(c_index, bounds.cols()) = bounds.row(0);
                    _ubA.segment(c_index, bounds.cols()) = bounds.row(1);
                    c_index += mat.rows();
                }
            }

            bool IDSolver::_solve()
            {
                if (_solver != nullptr)
                    return _solver->solve(_H, _g, _A, _lb, _ub, _lbA, _ubA);
                // else
                // ERROR
                return false;
            }
        } // namespace solver
    } // namespace dyn
} // namespace whc