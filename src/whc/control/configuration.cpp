//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Author/Maintainer:  Konstantinos Chatzilygeroudis
//|    email:   konstantinos.chatzilygeroudis@epfl.ch
//|    website: lasa.epfl.ch
//|
//|    This file is part of whc.
//|
//|    whc is free software: you can redistribute it and/or modify
//|    it under the terms of the GNU General Public License as published by
//|    the Free Software Foundation, either version 3 of the License, or
//|    (at your option) any later version.
//|
//|    whc is distributed in the hope that it will be useful,
//|    but WITHOUT ANY WARRANTY; without even the implied warranty of
//|    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//|    GNU General Public License for more details.
//|
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

        void Configuration::add_eef(const std::string& body_name, bool update_contact)
        {
            // Add end-effector only if in skeleton
            auto bd = _skeleton->getBodyNode(body_name);
            if (bd) {
                utils::EndEffector eef;
                eef.body_name = body_name;
                eef.update(_skeleton, update_contact);
                _end_effectors.push_back(eef);
            }
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