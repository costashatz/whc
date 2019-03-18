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

// associated header
#include "qld.h"

namespace Eigen {
    QLD::QLD() : A_(),
                 B_(),
                 fdOut_(0),
                 verbose_(false),
                 fail_(0),
                 U_(),
                 WAR_(),
                 IWAR_()
    {
    }

    QLD::QLD(int nrvar, int nreq, int nrineq, bool verbose) : A_(),
                                                              B_(),
                                                              fdOut_(0),
                                                              verbose_(verbose ? 1 : 0),
                                                              fail_(0),
                                                              U_(),
                                                              WAR_(),
                                                              IWAR_()
    {
        problem(nrvar, nreq, nrineq);
    }

    void QLD::fdOut(int fd)
    {
        fdOut_ = fd;
    }

    int QLD::fdOut() const
    {
        return fdOut_;
    }

    void QLD::verbose(bool v)
    {
        verbose_ = v ? 1 : 0;
    }

    bool QLD::verbose() const
    {
        return verbose_;
    }

    int QLD::fail() const
    {
        return fail_;
    }

    void QLD::problem(int nrvar, int nreq, int nrineq)
    {
        int nrconstr = nreq + nrineq;

        int MMAX = nrconstr == 0 ? 1 : nrconstr;
        // nrvar can't be == 0 so NMAX == nrvar
        int NMAX = nrvar;

        A_.resize(MMAX, NMAX);
        B_.resize(MMAX);

        X_.resize(nrvar);

        U_.resize(nrconstr + 2 * nrvar);
        int LWAR = static_cast<int>(
            std::ceil((3. * NMAX * NMAX) / 2. + 10. * NMAX + MMAX + nrconstr + 1.));
        WAR_.resize(LWAR);
        IWAR_.resize(nrvar);
    }

    const VectorXd& QLD::result() const
    {
        return X_;
    }

    const VectorXd& QLD::multipliers() const
    {
        return U_;
    }
} // namespace Eigen