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
#ifndef WHC_UTILS_COMMON_HPP
#define WHC_UTILS_COMMON_HPP

#include <string>

#include <Eigen/Core>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/Skeleton.hpp>

namespace whc {
    namespace utils {
        struct Contact {
            Eigen::VectorXd nz; // normal of contact - pointing towards the robot
            Eigen::VectorXd nx, ny; // orthonomal basis for contact
            Eigen::VectorXd min, max; // 6D vectors for lower and upper bounds
            double mu_s, mu_k; // coefficient of friction: static and kinetic
            // CoP constraints -- for humanoids
            bool cop_constraint = false; // enable/disable the CoP constraint
            double d_y_min, d_y_max, d_x_min, d_x_max; // rectangle boundaries for CoP constraint
        };

        struct Frame {
            Frame() {}
            Frame(const Frame& other) : pose(other.pose), vel(other.vel), acc(other.acc) {}

            Eigen::VectorXd pose;
            Eigen::VectorXd vel;
            Eigen::VectorXd acc;
        };

        struct ControlFrame : public Frame {
            ControlFrame() {}
            ControlFrame(const ControlFrame& other) : Frame(other), control_pose(other.control_pose), control_vel(other.control_vel), control_acc(other.control_acc) {}
            ControlFrame(const Frame& other) : Frame(other) {}

            bool control_pose = false;
            bool control_vel = false;
            bool control_acc = false;
        };

        struct EndEffector {
            std::string body_name;

            Frame state;
            ControlFrame desired;

            Contact contact;
            bool keep_contact = false;
            bool in_contact = false;

            void update(const dart::dynamics::SkeletonPtr& skeleton, bool update_contact = true)
            {
                auto bd = skeleton->getBodyNode(body_name);
                if (bd) {
                    Eigen::Isometry3d bd_trans = bd->getWorldTransform();
                    state.pose.resize(6);
                    // state.pose.head(3) = dart::math::matrixToEulerXYZ(bd_trans.linear().matrix());
                    state.pose.head(3) = dart::math::logMap(bd_trans.linear().matrix());
                    state.pose.tail(3) = bd_trans.translation();
                    state.vel = bd->getSpatialVelocity(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
                    state.acc = bd->getSpatialAcceleration(dart::dynamics::Frame::World(), dart::dynamics::Frame::World());

                    if (update_contact) {
                        Eigen::Vector3d bd_tr = bd_trans.translation();

                        Eigen::Isometry3d tx(Eigen::Isometry3d::Identity());
                        tx.translate(Eigen::Vector3d(1., 0., 0.));
                        dart::dynamics::SimpleFramePtr fx(new dart::dynamics::SimpleFrame(bd, "fx", tx));

                        contact.nx = (fx->getWorldTransform().translation() - bd_tr).normalized();

                        Eigen::Isometry3d ty(Eigen::Isometry3d::Identity());
                        ty.translate(Eigen::Vector3d(0., 1., 0.));
                        dart::dynamics::SimpleFramePtr fy(new dart::dynamics::SimpleFrame(bd, "fy", ty));

                        contact.ny = (fy->getWorldTransform().translation() - bd_tr).normalized();

                        Eigen::Isometry3d tz(Eigen::Isometry3d::Identity());
                        tz.translate(Eigen::Vector3d(0., 0., 1.));
                        dart::dynamics::SimpleFramePtr fz(new dart::dynamics::SimpleFrame(bd, "fz", tz));

                        contact.nz = (fz->getWorldTransform().translation() - bd_tr).normalized();

                        // TO-DO: Check if it is in contact
                    }
                }
            }
        };
    } // namespace utils
} // namespace whc

#endif