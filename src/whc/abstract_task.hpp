#ifndef WHC_ABSTRACT_TASK_HPP
#define WHC_ABSTRACT_TASK_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    class AbstractTask {
    public:
        AbstractTask() {}
        AbstractTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights) : _skeleton(skeleton), _desired(desired), _weights(weights) {}
        virtual ~AbstractTask() {}

        virtual std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() = 0;
        virtual std::string get_type() const = 0;
        virtual bool check_consistency() const { return ((_weights.size() == _desired.size()) || _weights.size() == 0 || _desired.size() == 0); }

        void set_weights(const Eigen::VectorXd& weights)
        {
            _weights = weights;
            assert(check_consistency());
        }

        Eigen::VectorXd get_weights() const { return _weights; }

        void set_desired(const Eigen::VectorXd& desired)
        {
            _desired = desired;
            assert(check_consistency());
        }

        Eigen::VectorXd get_desired() const { return _desired; }

    protected:
        dart::dynamics::SkeletonPtr _skeleton = nullptr;
        Eigen::VectorXd _desired, _weights;
    };
} // namespace whc

#endif