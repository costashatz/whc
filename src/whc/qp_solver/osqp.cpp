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
#ifdef USE_OSQP
#include <memory>

#include <OsqpEigen/Solver.hpp>

#include <whc/qp_solver/osqp.hpp>

namespace whc {
    namespace qp_solver {
        OSQP::OSQP(int max_iters, bool verbose) : _max_iters(max_iters), _verbose(verbose), _first(true)
        {
            reset();
        }

        OSQP::~OSQP() {}

        void OSQP::reset()
        {
            _first = true;
            if (_qp_solver)
                _qp_solver->clearSolver();
            _qp_solver.reset(new OsqpEigen::Solver());

            _qp_solver->settings()->setVerbosity(_verbose);
            _qp_solver->settings()->setWarmStart(true);
            _qp_solver->settings()->setMaxIteration(_max_iters);
        }

        bool OSQP::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            assert(H.rows() == H.cols());
            assert(H.rows() == g.size());
            assert(lb.size() == ub.size() && ub.size() == g.size());
            assert(A.cols() == g.size());
            assert(A.rows() == lbA.size() && lbA.size() == ubA.size());

            size_t dim = g.size();
            size_t num_constraints = A.rows() + dim; // bounds are treated as constraints

            Eigen::MatrixXd A_new = A;
            A_new.conservativeResize(num_constraints, A.cols());
            for (size_t i = A.rows(); i < num_constraints; i++) {
                A_new.row(i).setZero();
                A_new(i, i - A.rows()) = 1.;
            }

            _lb = lbA;
            _lb.conservativeResize(num_constraints);
            _lb.tail(dim) = lb;

            _ub = ubA;
            _ub.conservativeResize(num_constraints);
            _ub.tail(dim) = ub;

            // set the initial data of the QP solver
            _qp_solver->data()->setNumberOfVariables(dim);
            _qp_solver->data()->setNumberOfConstraints(num_constraints);

            Eigen::SparseMatrix<double> hessian = H.sparseView();
            Eigen::SparseMatrix<double> constraint = A_new.sparseView();

            _g = g;

            if (_first) {
                if (!_qp_solver->data()->setHessianMatrix(hessian))
                    return false;
                if (!_qp_solver->data()->setGradient(_g))
                    return false;
                if (!_qp_solver->data()->setLinearConstraintsMatrix(constraint))
                    return false;
                if (!_qp_solver->data()->setLowerBound(_lb))
                    return false;
                if (!_qp_solver->data()->setUpperBound(_ub))
                    return false;

                // instantiate the solver
                if (!_qp_solver->initSolver())
                    return false;

                _first = false;
            }
            else {
                if (!_qp_solver->updateHessianMatrix(hessian))
                    return false;
                if (!_qp_solver->updateGradient(_g))
                    return false;
                if (!_qp_solver->updateLinearConstraintsMatrix(constraint))
                    return false;
                if (!_qp_solver->updateBounds(_lb, _ub))
                    return false;
            }

            if (!_qp_solver->solve() || _qp_solver->getSolution().size() != static_cast<int>(dim))
                return false;

            _solution = _qp_solver->getSolution();

            return true;
        }

        Eigen::VectorXd OSQP::get_solution() const
        {
            return _solution;
        }

        void OSQP::set_max_iters(int max_iters)
        {
            _max_iters = max_iters;
        }

        double OSQP::get_max_iters() const
        {
            return _max_iters;
        }

        void OSQP::set_verbose(bool enable)
        {
            _verbose = enable;
        }

        bool OSQP::get_verbose() const
        {
            return _verbose;
        }
    } // namespace qp_solver
} // namespace whc
#endif