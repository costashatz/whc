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
#ifndef WHC_ABSTRACT_CONSTRAINT_HPP
#define WHC_ABSTRACT_CONSTRAINT_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    class AbstractWhcSolver;

    class AbstractConstraint {
    public:
        AbstractConstraint() {}
        AbstractConstraint(const dart::dynamics::SkeletonPtr& skeleton) : _skeleton(skeleton) {}
        virtual ~AbstractConstraint() {}

        virtual std::pair<Eigen::MatrixXd, Eigen::MatrixXd> data(AbstractWhcSolver*) = 0;

        virtual size_t N() const = 0;
        virtual std::string get_type() const = 0;

    protected:
        dart::dynamics::SkeletonPtr _skeleton = nullptr;
    };

    class AbstractContactConstraint : public AbstractConstraint {
    public:
        AbstractContactConstraint() {}
        AbstractContactConstraint(const dart::dynamics::SkeletonPtr& skeleton, const std::string& body_name) : AbstractConstraint(skeleton), _body_name(body_name) {}
        virtual ~AbstractContactConstraint() {}

        virtual Eigen::MatrixXd get_jacobian() const = 0;
        virtual Eigen::MatrixXd get_force_limits() const = 0;

    protected:
        std::string _body_name;
    };
} // namespace whc

#endif