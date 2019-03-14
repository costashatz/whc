#ifndef ICUB_TASK_TASK_HPP
#define ICUB_TASK_TASK_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    namespace task {
        class AbstractTask {
        public:
            AbstractTask() {}
            AbstractTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& weights) : _skeleton(skeleton), _weights(weights) {}

            virtual std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() = 0;
            virtual std::string get_type() const = 0;

        protected:
            dart::dynamics::SkeletonPtr _skeleton = nullptr;
            Eigen::VectorXd _weights;
        };

        class AccelerationTask : public AbstractTask {
        public:
            AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, double weight = 1.)
                : AccelerationTask(skeleton, body_name, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
            AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;

            std::string get_type() const override;

        protected:
            std::string _body_name;
            Eigen::VectorXd _desired_accelerations;

            Eigen::MatrixXd _jacobian, _jacobian_deriv;
            Eigen::VectorXd _dq;

            void _calculate_jacobian();
        };

        class COMAccelerationTask : public AbstractTask {
        public:
            COMAccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, double weight = 1.)
                : COMAccelerationTask(skeleton, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
            COMAccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;

            std::string get_type() const override;

        protected:
            std::string _body_name;
            Eigen::VectorXd _desired_accelerations;

            Eigen::MatrixXd _jacobian, _jacobian_deriv;
            Eigen::VectorXd _dq;

            void _calculate_jacobian();
        };

        class DirectTrackingTask : public AbstractTask {
        public:
            DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, double weight = 1.)
                : DirectTrackingTask(skeleton, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
            DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;

            std::string get_type() const override;

        protected:
            Eigen::VectorXd _desired_values;
        };

        class TauDiffTask : public AbstractTask {
        public:
            TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau, double weight = 1.)
                : TauDiffTask(skeleton, prev_tau, Eigen::VectorXd::Constant(prev_tau.size(), weight)) {}
            TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau, const Eigen::VectorXd& weights);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
            std::string get_type() const override;

        protected:
            Eigen::VectorXd _prev_tau;
        };
    } // namespace task
} // namespace whc

#endif