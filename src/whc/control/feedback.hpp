#ifndef WHC_CONTROL_FEEDBACK_HPP
#define WHC_CONTROL_FEEDBACK_HPP

#include <whc/utils/common.hpp>

namespace whc {
    namespace control {
        struct PDGains {
            Eigen::VectorXd kp, kd;
        };

        Eigen::VectorXd feedback(const utils::Frame& state, const utils::ControlFrame& desired, const PDGains& gains);
    } // namespace control
} // namespace whc

#endif