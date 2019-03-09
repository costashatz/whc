#ifndef ICUB_TASK_TASK_HPP
#define ICUB_TASK_TASK_HPP

#include <icub/model/iCub.hpp>

namespace icub {
    namespace task {
        template <typename Task, typename... Args>
        std::unique_ptr<Task> create_task(Args... args)
        {
            return std::unique_ptr<Task>(new Task(std::forward<Args>(args)...));
        }

        class AbstractTask {
        public:
            virtual std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() = 0;
            virtual std::string get_type() const = 0;
            virtual Eigen::MatrixXd get_jacobian() const = 0;
        };

        class AccelerationTask : public AbstractTask {
        public:
            AccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name, const Eigen::VectorXd& desired)
                : _skeleton(skeleton), _body_name(body_name), _desired_accelerations(desired) {}

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override
            {
                _calculate_jacobian();
                // Returns A matrix with only the acceleration part
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = _jacobian;
                Eigen::VectorXd b = _desired_accelerations - _jacobian_deriv * _dq;

                return std::make_pair(A, b);
            }

            Eigen::MatrixXd get_jacobian() const
            {
                return _jacobian;
            }

            std::string get_type() const
            {
                return "acceleration";
            }

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::string _body_name;
            Eigen::VectorXd _desired_accelerations;

            Eigen::MatrixXd _jacobian, _jacobian_deriv;
            Eigen::VectorXd _dq;

            void _calculate_jacobian()
            {
                auto bd = _skeleton->getBodyNode(_body_name);
                if (!bd)
                    return;

                _jacobian = _skeleton->getWorldJacobian(bd);
                _jacobian_deriv = _skeleton->getJacobianSpatialDeriv(bd, dart::dynamics::Frame::World());

                _dq = _skeleton->getVelocities();
            }
        };

        class COMAccelerationTask : public AbstractTask {
        public:
            COMAccelerationTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired)
                : _skeleton(skeleton), _desired_accelerations(desired) {}

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override
            {
                _calculate_jacobian();
                // Returns A matrix with only the acceleration part
                // the QPSolver should fix/resize the matrices
                Eigen::MatrixXd A = _jacobian;
                Eigen::VectorXd b = _desired_accelerations - _jacobian_deriv * _dq;

                return std::make_pair(A, b);
            }

            Eigen::MatrixXd get_jacobian() const
            {
                return _jacobian;
            }

            std::string get_type() const
            {
                return "com_acceleration";
            }

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::string _body_name;
            Eigen::VectorXd _desired_accelerations;

            Eigen::MatrixXd _jacobian, _jacobian_deriv;
            Eigen::VectorXd _dq;

            void _calculate_jacobian()
            {
                _jacobian = _skeleton->getCOMJacobian();
                _jacobian_deriv = _skeleton->getCOMJacobianSpatialDeriv();

                _dq = _skeleton->getVelocities();
            }
        };

        class DirectTrackingTask : public AbstractTask {
        public:
            DirectTrackingTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired)
                : _skeleton(skeleton), _desired_values(desired) {}

            std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() override
            {
                // Assumes desired values to be same size of optimization variables
                Eigen::MatrixXd A = Eigen::MatrixXd::Identity(_desired_values.size(), _desired_values.size());
                Eigen::VectorXd b = _desired_values;

                return std::make_pair(A, b);
            }

            std::string get_type() const
            {
                return "direct";
            }

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            Eigen::VectorXd _desired_values;
        };
    } // namespace task
} // namespace icub

#endif