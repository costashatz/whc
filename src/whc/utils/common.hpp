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
            double mu, muR; // coefficient of friction: static and kinetic
            double min_force, max_force;
            double d_y_min, d_y_max, d_x_min, d_x_max;
            bool calculate_torque = false;
            Eigen::VectorXd min, max; // 6D vectors for lower and upper bounds
        };

        struct Frame {
            Eigen::VectorXd pose;
            Eigen::VectorXd vel;
            Eigen::VectorXd acc;
        };

        struct ControlFrame : public Frame {
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
                    state.pose = dart::math::logMap(bd->getWorldTransform());
                    state.vel = bd->getSpatialVelocity();
                    state.acc = bd->getSpatialAcceleration();

                    if (update_contact) {
                        Eigen::Vector3d bd_tr = bd->getWorldTransform().translation();

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