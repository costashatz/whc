#ifndef ICUB_SOLVER_QP_SOLVER_HPP
#define ICUB_SOLVER_QP_SOLVER_HPP

#include <qpOASES.hpp>

namespace icub {
    namespace solver {
        template <typename QPProblem>
        class QPSolver {
        public:
            QPSolver() {}

        protected:
            QPProblem _solver;
        };
    } // namespace solver
} // namespace icub

#endif