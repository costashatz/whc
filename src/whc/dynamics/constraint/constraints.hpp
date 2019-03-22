#ifndef WHC_DYNAMICS_CONSTRAINT_CONSTRAINTS_HPP
#define WHC_DYNAMICS_CONSTRAINT_CONSTRAINTS_HPP

#include <whc/abstract_constraint.hpp>
#include <whc/abstract_whc_solver.hpp>
#include <whc/utils/common.hpp>

namespace whc {
    namespace dyn {
        namespace constraint {
            class DynamicsConstraint : public AbstractConstraint {
            public:
                DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base = true);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                size_t N() const override;
                std::string get_type() const override;

            protected:
                bool _floating_base;
            };

            class ContactConstraint : public AbstractContactConstraint {
            public:
                ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const utils::Contact& contact);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                Eigen::MatrixXd get_jacobian() const;
                Eigen::MatrixXd get_force_limits() const;

                size_t N() const override;
                std::string get_type() const override;

            protected:
                utils::Contact _contact;
            };

            class JointLimitsConstraint : public AbstractConstraint {
            public:
                JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton);

                std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver* solver) override;

                size_t N() const override;
                std::string get_type() const override;
            };
        } // namespace constraint
    } // namespace dyn
} // namespace whc

#endif