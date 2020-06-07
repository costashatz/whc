//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Author/Maintainer:  Konstantinos Chatzilygeroudis
//|    email:   konstantinos.chatzilygeroudis@epfl.ch
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
#ifndef WHC_QP_SOLVER_OSQP_HPP
#define WHC_QP_SOLVER_OSQP_HPP

#include <whc/qp_solver/abstract_qp.hpp>

#include <Eigen/Sparse>

namespace OsqpEigen {
    class Solver;
}

namespace whc {
    namespace qp_solver {
        class OSQP : public AbstractQP {
        public:
            OSQP(int max_iters = 1000, bool verbose = true);
            virtual ~OSQP();

            bool solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA) override;
            Eigen::VectorXd get_solution() const override;

            void set_max_iters(int max_iters);
            double get_max_iters() const;

            void set_verbose(bool enable = true);
            bool get_verbose() const;

            void reset();

        protected:
            Eigen::VectorXd _solution;
            int _max_iters;
            bool _verbose, _first;
            std::unique_ptr<OsqpEigen::Solver> _qp_solver;

            // Eigen::SparseMatrix<double> _H, _A;
            Eigen::VectorXd _g, _lb, _ub;
        };
    } // namespace qp_solver
} // namespace whc

#endif