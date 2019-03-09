#ifndef ICUB_MODEL_ICUB_COMMON_HPP
#define ICUB_MODEL_ICUB_COMMON_HPP

#include <string>
#include <vector>

namespace icub {
    namespace model {
        std::vector<std::pair<std::string, std::string>> packages();

        std::vector<std::string> bodies();

        std::vector<std::string> leg_eefs();
        std::vector<std::string> arm_eefs();

        std::string base_link();
        std::string urdf_fake_base_link();
        std::string head_link();

        std::vector<std::string> joints();
        std::vector<std::string> actuated_joints();
    } // namespace model
} // namespace icub

#endif