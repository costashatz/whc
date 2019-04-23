#ifndef WHC_KINEMATICS_SOLVER_IK_SOLVER_HPP
#define WHC_KINEMATICS_SOLVER_IK_SOLVER_HPP

#include <dart/dynamics/Skeleton.hpp>

#include <whc/abstract_whc_solver.hpp>
#include <whc/kinematics/constraint/constraints.hpp>
#include <whc/kinematics/task/tasks.hpp>
#include <whc/utils/helpers.hpp>

namespace whc {
    namespace kin {
        namespace solver {
            class IKSolver : public AbstractWhcSolver {
            public:
                IKSolver();
                IKSolver(const dart::dynamics::SkeletonPtr& skeleton);

                bool solve() override;

            protected:
                void _setup_matrices();
                bool _solve();
            };
        } // namespace solver
    } // namespace kin
} // namespace whc

#endif