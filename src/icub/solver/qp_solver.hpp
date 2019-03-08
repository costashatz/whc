#ifndef ICUB_SOLVER_QP_SOLVER_HPP
#define ICUB_SOLVER_QP_SOLVER_HPP

#include <qpOASES.hpp>

#include <icub/model/iCub.hpp>

namespace icub {
    namespace solver {
        template <typename QPProblem>
        class QPSolver {
        public:
            QPSolver() {}
            QPSolver(model::iCub* icub) : _icub(icub) {}

            void set_icub(model::iCub* icub)
            {
                _icub = icub;
            }

        protected:
            QPProblem _solver;
            model::iCub* _icub;
        };
    } // namespace solver
} // namespace icub

#endif