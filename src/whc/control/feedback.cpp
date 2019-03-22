#include <whc/control/feedback.hpp>
#include <whc/utils/math.hpp>

namespace whc {
    namespace control {
        Eigen::VectorXd feedback(const utils::Frame& state, const utils::ControlFrame& desired, const PDGains& gains)
        {
            Eigen::VectorXd pos_error = desired.pose - state.pose;
            pos_error.head(3) = utils::rotation_error(dart::math::expMapRot(desired.pose.head(3)), dart::math::expMapRot(state.pose.head(3)));

            return desired.acc + gains.kd.cwiseProduct(desired.vel - state.vel) + gains.kp.cwiseProduct(pos_error);
        }
    } // namespace control
} // namespace whc