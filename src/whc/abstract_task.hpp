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
#ifndef WHC_ABSTRACT_TASK_HPP
#define WHC_ABSTRACT_TASK_HPP

#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    class AbstractTask {
    public:
        AbstractTask() {}
        AbstractTask(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& desired, const Eigen::VectorXd& weights) : _skeleton(skeleton), _desired(desired), _weights(weights) {}
        virtual ~AbstractTask() {}

        virtual std::pair<Eigen::MatrixXd, Eigen::VectorXd> get_costs() = 0;
        virtual std::string get_type() const = 0;
        virtual bool check_consistency() const { return ((_weights.size() == _desired.size()) || _weights.size() == 0 || _desired.size() == 0); }

        void set_weights(const Eigen::VectorXd& weights)
        {
            _weights = weights;
            assert(check_consistency());
        }

        Eigen::VectorXd get_weights() const { return _weights; }

        void set_desired(const Eigen::VectorXd& desired)
        {
            _desired = desired;
            assert(check_consistency());
        }

        Eigen::VectorXd get_desired() const { return _desired; }

    protected:
        dart::dynamics::SkeletonPtr _skeleton = nullptr;
        Eigen::VectorXd _desired, _weights;
    };
} // namespace whc

#endif