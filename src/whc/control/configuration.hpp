#ifndef WHC_CONTROL_CONFIGURATION_HPP
#define WHC_CONTROL_CONFIGURATION_HPP

#include <vector>

#include <whc/utils/common.hpp>

namespace whc {
    namespace control {
        class Configuration {
        public:
            Configuration(const dart::dynamics::SkeletonPtr& skeleton = nullptr);

            dart::dynamics::SkeletonPtr skeleton();
            void set_skeleton(const dart::dynamics::SkeletonPtr& skeleton);

            void clear_all();

            void add_eef(const utils::EndEffector& eef);

            utils::EndEffector* eef(size_t index);
            const utils::EndEffector* eef(size_t index) const;

            utils::EndEffector* eef(const std::string& body_name);
            const utils::EndEffector* eef(const std::string& body_name) const;

            void update(bool update_contacts = true);

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::vector<utils::EndEffector> _end_effectors;
        };
    } // namespace control
} // namespace whc

#endif