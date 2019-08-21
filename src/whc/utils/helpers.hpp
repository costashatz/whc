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
#ifndef WHC_UTILS_HELPERS_HPP
#define WHC_UTILS_HELPERS_HPP

#include <memory>

namespace whc {
    namespace utils {
        template <typename T, typename... Args>
        std::unique_ptr<T> make_unique(Args... args)
        {
            return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
        }
    } // namespace utils
} // namespace whc

#endif