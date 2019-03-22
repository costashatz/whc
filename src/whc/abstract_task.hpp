#ifndef WHC_ABSTRACT_TASK_HPP
#define WHC_ABSTRACT_TASK_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    class AbstractTask {
    public:
        AbstractTask() {}
        AbstractTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& weights) : _skeleton(skeleton), _weights(weights) {}
        virtual ~AbstractTask() {}

        virtual std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() = 0;
        virtual std::string get_type() const = 0;

    protected:
        dart::dynamics::SkeletonPtr _skeleton = nullptr;
        Eigen::VectorXd _weights;
    };
} // namespace whc

#endif