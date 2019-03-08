#ifndef ICUB_TASK_TASK_HPP
#define ICUB_TASK_TASK_HPP

#include <icub/model/iCub.hpp>

namespace icub {
    namespace task {
        class AbstractTask {
        public:
        };

        class AccelerationTask : public AbstractTask {
        public:
        protected:
            Eigen::VectorXd _desired_accelerations;
        };
    } // namespace task
} // namespace icub

#endif