#ifndef WHC_CONSTRAINT_CONSTRAINT_HPP
#define WHC_CONSTRAINT_CONSTRAINT_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    namespace solver {
        class WhcSolver;
    }
    namespace constraint {
        class AbstractConstraint {
        public:
            virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::WhcSolver&, size_t index = 0) = 0;

            virtual size_t N() const = 0;
            virtual std::string get_type() const = 0;
        };

        class DynamicsConstraint : public AbstractConstraint {
        public:
            DynamicsConstraint(const dart::dynamics::SkeletonPtr& skeleton, bool floating_base = true);

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::WhcSolver& solver, size_t index = 0);

            size_t N() const;
            std::string get_type() const;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            bool _floating_base;
        };

        struct Contact {
            Eigen::VectorXd normal; // pointing towards the robot
            Eigen::VectorXd t1, t2; // orthonomal basis for contact
            double mu, muR;
            double min_force, max_force;
            double d_y_min, d_y_max, d_x_min, d_x_max;
            bool calculate_torque = false;
            Eigen::VectorXd min, max; // 6D vectors for lower and upper bounds
        };

        class ContactConstraint : public AbstractConstraint {
        public:
            ContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Contact& contact);

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::WhcSolver& solver, size_t index);

            Eigen::MatrixXd get_jacobian() const;
            Eigen::MatrixXd get_force_limits() const;

            size_t N() const;
            std::string get_type() const;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::string _body_name;
            Contact _contact;
        };

        class JointLimitsConstraint : public AbstractConstraint {
        public:
            JointLimitsConstraint(const dart::dynamics::SkeletonPtr& skeleton);

            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(solver::WhcSolver& solver, size_t index = 0);

            size_t N() const;
            std::string get_type() const;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
        };
    } // namespace constraint
} // namespace whc

#endif