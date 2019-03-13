#ifndef ICUB_TASK_TASK_HPP
#define ICUB_TASK_TASK_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    namespace task {
        class AbstractTask {
        public:
            virtual std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() = 0;
            virtual std::string get_type() const = 0;
        };

        class AccelerationTask : public AbstractTask {
        public:
            AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;

            std::string get_type() const override;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::string _body_name;
            Eigen::VectorXd _desired_accelerations;

            Eigen::MatrixXd _jacobian, _jacobian_deriv;
            Eigen::VectorXd _dq;

            void _calculate_jacobian();
        };

        class COMAccelerationTask : public AbstractTask {
        public:
            COMAccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;

            std::string get_type() const override;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::string _body_name;
            Eigen::VectorXd _desired_accelerations;

            Eigen::MatrixXd _jacobian, _jacobian_deriv;
            Eigen::VectorXd _dq;

            void _calculate_jacobian();
        };

        class DirectTrackingTask : public AbstractTask {
        public:
            DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;

            std::string get_type() const override;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            Eigen::VectorXd _desired_values;
        };

        class TauDiffTask : public AbstractTask {
        public:
            TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau);

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
            std::string get_type() const override;

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            Eigen::VectorXd _prev_tau;
        };
    } // namespace task
} // namespace whc

#endif