#ifndef WHC_DYNAMICS_TASK_TASKS_HPP
#define WHC_DYNAMICS_TASK_TASKS_HPP

#include <whc/abstract_task.hpp>

namespace whc {
    namespace dyn {
        namespace task {
            class AccelerationTask : public AbstractTask {
            public:
                AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, double weight = 1.)
                    : AccelerationTask(skeleton, body_name, desired, Eigen::VectorXd::Constant(desired.size(), weight)) {}
                AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;

            protected:
                std::string _body_name;

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
            };

            class TauDiffTask : public AbstractTask {
            public:
                TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau, double weight = 1.)
                    : TauDiffTask(skeleton, prev_tau, Eigen::VectorXd::Constant(prev_tau.size(), weight)) {}
                TauDiffTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& prev_tau, const Eigen::VectorXd& weights);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;
            };

            class PostureTask : public AbstractTask {
            public:
                PostureTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, double weight = 1., bool floating_base = true)
                    : PostureTask(skeleton, desired, Eigen::VectorXd::Constant(desired.size(), weight), floating_base) {}
                PostureTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights, bool floating_base = true);

                std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override;
                std::string get_type() const override;
                bool check_consistency() const override;

            protected:
                bool _floating_base;
            };
        } // namespace task
    } // namespace dyn
} // namespace whc

#endif