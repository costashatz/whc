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
#ifndef WHC_QP_SOLVER_QP_OASES_HPP
#define WHC_QP_SOLVER_QP_OASES_HPP

#include <whc/qp_solver/abstract_qp.hpp>

namespace qpOASES {
    class SQProblem;
}

namespace whc {
    namespace qp_solver {
        class QPOases : public AbstractQP {
        public:
            QPOases(double max_time = 0.005, int max_iters = 1000, bool verbose = true);
            virtual ~QPOases();

            bool solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA) override;
            Eigen::VectorXd get_solution() const override;

            void set_max_time(double max_time);
            double get_max_time() const;

            void set_max_iters(int max_iters);
            double get_max_iters() const;

            void set_verbose(bool enable = true);
            bool get_verbose() const;

        protected:
            Eigen::VectorXd _solution;
            double _max_time;
            int _max_iters;
            bool _verbose;
            std::unique_ptr<qpOASES::SQProblem> _qp_solver;
        };
    } // namespace qp_solver
} // namespace whc

#endif