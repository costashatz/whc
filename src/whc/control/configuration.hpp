//|
//|    Copyright (C) 2019 Learning Algorithms and Systems Laboratory, EPFL, Switzerland
//|    Author/Maintainer:  Konstantinos Chatzilygeroudis
//|    email:   costashatz@gmail.com
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
            void add_eef(const std::string& body_name, bool update_contact = true);

            utils::EndEffector* eef(size_t index);
            const utils::EndEffector* eef(size_t index) const;

            utils::EndEffector* eef(const std::string& body_name);
            const utils::EndEffector* eef(const std::string& body_name) const;

            size_t num_eefs() const;

            void update(bool update_contacts = true);

        protected:
            dart::dynamics::SkeletonPtr _skeleton;
            std::vector<utils::EndEffector> _end_effectors;
        };
    } // namespace control
} // namespace whc

#endif