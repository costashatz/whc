#ifndef WHC_ABSTRACT_CONSTRAINT_HPP
#define WHC_ABSTRACT_CONSTRAINT_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    class AbstractWhcSolver;

    class AbstractConstraint {
    public:
        AbstractConstraint() {}
        AbstractConstraint(const dart::dynamics::SkeletonPtr& skeleton) : _skeleton(skeleton) {}
        virtual ~AbstractConstraint() {}

        virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver*) = 0;

        virtual size_t N() const = 0;
        virtual std::string get_type() const = 0;

    protected:
        dart::dynamics::SkeletonPtr _skeleton = nullptr;
    };

    class AbstractContactConstraint : public AbstractConstraint {
    public:
        AbstractContactConstraint() {}
        AbstractContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name) : AbstractConstraint(skeleton), _body_name(body_name) {}
        virtual ~AbstractContactConstraint() {}

        virtual Eigen::MatrixXd get_jacobian() const = 0;
        virtual Eigen::MatrixXd get_force_limits() const = 0;

    protected:
        std::string _body_name;
    };
} // namespace whc

#endif