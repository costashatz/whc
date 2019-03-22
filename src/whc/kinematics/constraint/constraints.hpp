#ifndef WHC_KINEMATICS_CONSTRAINT_CONSTRAINTS_HPP
#define WHC_KINEMATICS_CONSTRAINT_CONSTRAINTS_HPP

#include <whc/abstract_constraint.hpp>
#include <whc/abstract_whc_solver.hpp>
#include <whc/utils/common.hpp>

namespace whc {
    namespace kin {
        namespace constraint {
            class JointLimitsConstraint : public AbstractConstraint {
            public:
                JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                size_t N() const override;
                std::string get_type() const override;
            };
        } // namespace constraint
    } // namespace kin
} // namespace whc

#endif