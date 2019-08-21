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
#include <memory>

#include <qpOASES.hpp>

#include <whc/qp_solver/qp_oases.hpp>

namespace whc {
    namespace qp_solver {
        QPOases::QPOases(double max_time, int max_iters, bool verbose) : _max_time(max_time), _max_iters(max_iters), _verbose(verbose), _qp_solver(nullptr) {}

        QPOases::~QPOases() {}

        bool QPOases::solve(const Eigen::MatrixXd& H, const Eigen::VectorXd& g, const Eigen::MatrixXd& A, const Eigen::VectorXd& lb, const Eigen::VectorXd& ub, const Eigen::VectorXd& lbA, const Eigen::VectorXd& ubA)
        {
            assert(H.rows() == H.cols());
            assert(H.rows() == g.size());
            assert(lb.size() == ub.size() && ub.size() == g.size());
            assert(A.cols() == g.size());
            assert(A.rows() == lbA.size() && lbA.size() == ubA.size());

            size_t dim = g.size();
            size_t num_constraints = A.rows();
            bool first = false;
            if (!_qp_solver) {
                _qp_solver = std::unique_ptr<qpOASES::SQProblem>(new qpOASES::SQProblem(dim, num_constraints));
                first = true;
            }

            auto options = _qp_solver->getOptions();
            options.printLevel = qpOASES::PL_NONE;
            if (_verbose)
                options.printLevel = qpOASES::PL_LOW;
            // options.enableFarBounds = qpOASES::BT_TRUE;
            // options.enableFlippingBounds = qpOASES::BT_TRUE;
            options.enableRamping = qpOASES::BT_FALSE;
            options.enableNZCTests = qpOASES::BT_FALSE;
            options.enableDriftCorrection = 0;
            options.terminationTolerance = 1e-6;
            options.boundTolerance = 1e-4;
            options.epsIterRef = 1e-6;

            _qp_solver->setOptions(options);
            // we need this, because QPOases changes these values
            double max_time = _max_time;
            int max_iters = _max_iters;

            // qpOASES uses row-major storing
            qpOASES::real_t* H_qp = new qpOASES::real_t[dim * dim];
            qpOASES::real_t* A_qp = new qpOASES::real_t[num_constraints * dim];
            qpOASES::real_t* g_qp = new qpOASES::real_t[dim];
            qpOASES::real_t* lb_qp = new qpOASES::real_t[dim];
            qpOASES::real_t* ub_qp = new qpOASES::real_t[dim];
            qpOASES::real_t* lbA_qp = new qpOASES::real_t[num_constraints];
            qpOASES::real_t* ubA_qp = new qpOASES::real_t[num_constraints];

            for (int i = 0; i < H.rows(); i++) {
                for (int j = 0; j < H.cols(); j++) {
                    H_qp[i * H.cols() + j] = H(i, j);
                }
            }

            for (int i = 0; i < A.rows(); i++) {
                for (int j = 0; j < A.cols(); j++) {
                    A_qp[i * A.cols() + j] = A(i, j);
                }
            }

            for (int i = 0; i < g.size(); i++) {
                g_qp[i] = g(i);
                lb_qp[i] = lb(i);
                ub_qp[i] = ub(i);
            }

            for (size_t i = 0; i < num_constraints; i++) {
                lbA_qp[i] = lbA(i);
                ubA_qp[i] = ubA(i);
            }

            qpOASES::SymSparseMat H_mat(H.rows(), H.cols(), H.cols(), H_qp);
            H_mat.createDiagInfo();
            qpOASES::SparseMatrix A_mat(A.rows(), A.cols(), A.cols(), A_qp);

            qpOASES::returnValue ret = qpOASES::TERMINAL_LIST_ELEMENT;

            if (first)
                ret = _qp_solver->init(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters);
            else
                ret = _qp_solver->hotstart(&H_mat, g_qp, &A_mat, lb_qp, ub_qp, lbA_qp, ubA_qp, max_iters, &max_time);

            delete[] H_qp;
            delete[] A_qp;
            delete[] g_qp;
            delete[] lb_qp;
            delete[] ub_qp;
            delete[] lbA_qp;
            delete[] ubA_qp;

            _solution = Eigen::VectorXd(dim);
            _qp_solver->getPrimalSolution(_solution.data());

            if (ret != qpOASES::SUCCESSFUL_RETURN)
                return false;
            return true;
        }

        Eigen::VectorXd QPOases::get_solution() const
        {
            return _solution;
        }

        void QPOases::set_max_time(double max_time)
        {
            _max_time = max_time;
        }

        double QPOases::get_max_time() const
        {
            return _max_time;
        }

        void QPOases::set_max_iters(int max_iters)
        {
            _max_iters = max_iters;
        }

        double QPOases::get_max_iters() const
        {
            return _max_iters;
        }

        void QPOases::set_verbose(bool enable)
        {
            _verbose = enable;
        }

        bool QPOases::get_verbose() const
        {
            return _verbose;
        }
    } // namespace qp_solver
} // namespace whc