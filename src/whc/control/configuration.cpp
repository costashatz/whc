#include <whc/control/configuration.hpp>

namespace whc {
    namespace control {
        Configuration::Configuration(const dart::dynamics::SkeletonPtr& skeleton) : _skeleton(skeleton) {}

        dart::dynamics::SkeletonPtr Configuration::skeleton() { return _skeleton; }
        void Configuration::set_skeleton(const dart::dynamics::SkeletonPtr& skeleton) { _skeleton = skeleton; }

        void Configuration::clear_all() { _end_effectors.clear(); }

        void Configuration::add_eef(const utils::EndEffector& eef)
        {
            // Add end-effector only if in skeleton
            auto bd = _skeleton->getBodyNode(eef.body_name);
            if (bd)
                _end_effectors.push_back(eef);
        }

        utils::EndEffector* Configuration::eef(size_t index)
        {
            assert(index < _end_effectors.size());
            return &_end_effectors[index];
        }

        const utils::EndEffector* Configuration::eef(size_t index) const
        {
            assert(index < _end_effectors.size());
            return &_end_effectors[index];
        }

        utils::EndEffector* Configuration::eef(const std::string& body_name)
        {
            for (auto& e : _end_effectors) {
                if (e.body_name == body_name)
                    return &e;
            }

            return nullptr;
        }

        const utils::EndEffector* Configuration::eef(const std::string& body_name) const
        {
            for (const auto& e : _end_effectors) {
                if (e.body_name == body_name)
                    return &e;
            }

            return nullptr;
        }

        size_t Configuration::num_eefs() const
        {
            return _end_effectors.size();
        }

        void Configuration::update(bool update_contacts)
        {
            for (auto& e : _end_effectors) {
                e.update(_skeleton, update_contacts);
            }
        }
    } // namespace control
} // namespace whc