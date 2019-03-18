// Copyright 2015-2016 CNRS-UM LIRMM, CNRS-AIST JRL
//
// This file is part of eigen-qld.
//
// eigen-qld is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// eigen-qld is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with eigen-qld.  If not, see <http://www.gnu.org/licenses/>.

#ifndef QLD_EIGEN_QLD_H
#define QLD_EIGEN_QLD_H

// includes
// std
#include <cassert>

// Eigen
#include <Eigen/Core>

namespace Eigen {

    extern "C" int ql_(const int* m, const int* me, const int* mmax,
        const int* n, const int* nmax, const int* mnn,
        const double* c, const double* d, const double* a, const double* b,
        const double* xl, const double* xu,
        double* x, double* u, const double* eps, const int* mode,
        const int* iout, int* ifail, const int* iprint,
        double* war, int* lwar, int* iwar, int* liwar);

    class QLD {
    public:
        QLD();
        QLD(int nrvar, int nreq, int nrineq, bool verbose = false);

        void fdOut(int fd);
        int fdOut() const;

        void verbose(bool v);
        bool verbose() const;

        int fail() const;

        void problem(int nrvar, int nreq, int nrineq);

        const VectorXd& result() const;

        /** Return the lagrange multipliers associated with results **/
        const VectorXd& multipliers() const;

        template <typename MatObj, typename VecObj,
            typename MatEq, typename VecEq,
            typename MatIneq, typename VecIneq,
            typename VecVar>
        bool solve(const MatrixBase<MatObj>& Q, const MatrixBase<VecObj>& C,
            const MatrixBase<MatEq>& Aeq, const MatrixBase<VecEq>& Beq,
            const MatrixBase<MatIneq>& Aineq, const MatrixBase<VecIneq>& Bineq,
            const MatrixBase<VecVar>& XL, const MatrixBase<VecVar>& XU,
            bool isDecomp = false, double eps = 1e-12);

    private:
        MatrixXd A_;
        VectorXd B_, X_;
        int fdOut_;
        int verbose_;
        int fail_;
        VectorXd U_;
        VectorXd WAR_;
        VectorXi IWAR_;
        inline int fortran_ql(const int* m, const int* me, const int* mmax,
            const int* n, const int* nmax, const int* mnn,
            const double* c, const double* d, const double* a, const double* b,
            const double* xl, const double* xu,
            double* x, double* u, const double* eps, const int* mode,
            const int* iout, int* ifail, const int* iprint,
            double* war, int* lwar, int* iwar, int* liwar)
        {
            return ql_(m, me, mmax, n, nmax, mnn, c, d, a, b, xl, xu, x, u, eps, mode, iout, ifail, iprint, war, lwar, iwar, liwar);
        }
    };

    // inline
    template <typename MatObj, typename VecObj,
        typename MatEq, typename VecEq,
        typename MatIneq, typename VecIneq,
        typename VecVar>
    inline bool QLD::solve(const MatrixBase<MatObj>& Q, const MatrixBase<VecObj>& C,
        const MatrixBase<MatEq>& Aeq, const MatrixBase<VecEq>& Beq,
        const MatrixBase<MatIneq>& Aineq, const MatrixBase<VecIneq>& Bineq,
        const MatrixBase<VecVar>& XL, const MatrixBase<VecVar>& XU,
        bool isDecomp, double eps)
    {
        assert(Aeq.rows() == Beq.rows()); // check equality size
        assert(Aeq.cols() == X_.rows());
        assert(Aineq.rows() == Bineq.rows()); // check inequality size
        assert(Aineq.cols() == X_.rows());
        assert(Q.rows() == Q.cols()); // check Q is square
        assert(Q.cols() == X_.rows()); // check Q has the good number of variable
        assert(C.rows() == X_.rows()); // check C size
        assert(XU.rows() == X_.rows()); // check XL size
        assert(XL.rows() == X_.rows()); // check XU size

        int nreq = int(Beq.rows());
        int nrineq = int(Bineq.rows());
        int nrvar = int(X_.rows());

        int mode = isDecomp ? 0 : 1;

        int M = nreq + nrineq;
        int N = nrvar;

        int MMAX = int(A_.rows());
        // beware, don't work if base Q is not square
        int NMAX = int(Q.rows());

        int NMN = M + 2 * N;
        int LWAR = int(WAR_.rows());
        int LIWAR = int(IWAR_.rows());

        A_.block(0, 0, nreq, nrvar) = -Aeq;
        A_.block(nreq, 0, nrineq, nrvar) = -Aineq;

        B_.segment(0, nreq) = Beq;
        B_.segment(nreq, nrineq) = Bineq;

        fortran_ql(&M, &nreq, &MMAX, &N, &NMAX, &NMN,
            Q.derived().data(), C.derived().data(), A_.data(), B_.data(),
            XL.derived().data(), XU.derived().data(), X_.data(),
            U_.data(), &eps, &mode, &fdOut_, &fail_, &verbose_,
            WAR_.data(), &LWAR, IWAR_.data(), &LIWAR);

        return fail_ == 0;
    }
} // namespace Eigen

#endif
