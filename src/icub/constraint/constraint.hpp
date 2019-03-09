#ifndef ICUB_CONSTRAINT_CONSTRAINT_HPP
#define ICUB_CONSTRAINT_CONSTRAINT_HPP

#include <icub/model/iCub.hpp>

namespace icub {
    namespace solver {
        class QPSolver;
    }
    namespace constraint {
        template <typename Constraint, typename... Args>
        std::unique_ptr<Constraint> create_constraint(Args... args)
        {
            return std::unique_ptr<Constraint>(new Constraint(std::forward<Args>(args)...));
        }

        class AbstractConstraint {
        public:
            virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::QPSolver&) = 0;

            virtual size_t N() const = 0;
            virtual std::string get_type() const = 0;
        };

        class DynamicsConstraint : public AbstractConstraint {
        public:
            DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base = true);

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::QPSolver& solver);

            size_t N() const;
            std::string get_type() const;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            bool _floating_base;
        };

        class ContactConstraint : public AbstractConstraint {
        public:
            ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, double mu);

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::QPSolver& solver);

            Eigen::MatrixXd get_jacobian() const;

            size_t N() const;
            std::string get_type() const;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::string _body_name;
            double _mu;
        };
    } // namespace constraint
} // namespace icub

#endif