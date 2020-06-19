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
#include <iostream>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#ifdef GRAPHIC
#include <robot_dart/gui/magnum/graphics.hpp>
#endif

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/fcl/FCLCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <whc/control/configuration.hpp>
#include <whc/control/feedback.hpp>
#include <whc/dynamics/constraint/constraints.hpp>
#include <whc/dynamics/solver/id_solver.hpp>
#include <whc/dynamics/task/tasks.hpp>
#ifdef USE_OSQP
#include <whc/qp_solver/osqp.hpp>
#endif
#include <whc/qp_solver/qp_oases.hpp>
#include <whc/utils/math.hpp>

#include "iCub.hpp"

class QPControl : public robot_dart::control::RobotControl {
public:
    QPControl() : robot_dart::control::RobotControl() {}
    virtual ~QPControl() {}

    void configure() override
    {
        _active = true;
        auto robot = _robot.lock();
#if DART_VERSION_AT_LEAST(6, 7, 2)
        auto skel = robot->skeleton()->cloneSkeleton();
#else
        auto skel = robot->skeleton()->clone();
#endif
        _solver = std::make_shared<whc::dyn::solver::IDSolver>(skel);
#ifdef USE_OSQP
        _solver->set_qp_solver<whc::qp_solver::OSQP>(200, false);
#else
        _solver->set_qp_solver<whc::qp_solver::QPOases>(0.005, 1000, false);
#endif
        _prev_tau = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs());
        _init_pos = robot->skeleton()->getPositions();

        _config = whc::control::Configuration(skel);
        _config.add_eef("head", false); // no need for contacts for head
        _config.add_eef("root_link", false); // no need for contacts for root link
        _config.add_eef("chest", false); // no need for contacts for root link
        _config.add_eef("r_hand", false); // no need for contacts for hands
        _config.add_eef("l_hand", false);

        _config.add_eef("r_sole"); // contacts for feet
        _config.add_eef("l_sole");

        // Get positions/velocities from real robot
        Eigen::VectorXd pos = robot->skeleton()->getPositions();
        Eigen::VectorXd vel = robot->skeleton()->getVelocities();
        _config.skeleton()->setPositions(pos);
        _config.skeleton()->setVelocities(vel);

        _config.update(true); // update contact information as well

        // Set desired state -- keep end-effectors posture
        for (size_t i = 0; i < _config.num_eefs(); i++) {
            _config.eef(i)->desired = _config.eef(i)->state;
            _config.eef(i)->desired.vel = Eigen::VectorXd::Zero(6);
            _config.eef(i)->desired.acc = Eigen::VectorXd::Zero(6);
        }

        // Contact information for feet eefs
        whc::utils::Contact contact_right;
        contact_right.mu_s = 1.;
        contact_right.mu_k = 1.;
        double max_force = 500.;
        contact_right.min = Eigen::VectorXd::Zero(6);
        contact_right.max = Eigen::VectorXd::Zero(6);
        contact_right.max.tail(3) << max_force, max_force, max_force;
        double max_torque = 100.;
        contact_right.cop_constraint = true;
        // if (contact_right.cop_constraint) {
        contact_right.min.head(3) << -max_torque, -max_torque, -max_torque;
        contact_right.max.head(3) << max_torque, max_torque, max_torque;
        // }

        // iCub Nancy
        double foot_size_x = 0.12;
        double foot_size_y = 0.04;
        double sole_x = 0.04; //0.0214502260596 + 0.02;
        double sole_y = 0.;
        // iCub Gazebo
        if (robot->skeleton()->getName() != "iCubNancy01") {
            foot_size_x = 0.14;
            foot_size_y = 0.04;
            sole_x = 0.04;
            sole_y = 0.01;
        }
        contact_right.d_y_min = -foot_size_y / 2. - sole_y;
        contact_right.d_y_max = foot_size_y / 2.;
        contact_right.d_x_min = -sole_x;
        contact_right.d_x_max = foot_size_x - sole_x;

        whc::utils::Contact contact_left = contact_right;

        contact_left.d_y_max = contact_right.d_y_max + sole_y;
        contact_left.d_y_min = contact_right.d_y_min + sole_y;

        _config.eef("r_sole")->keep_contact = true;
        _config.eef("r_sole")->contact = contact_right;
        _config.eef("l_sole")->keep_contact = true;
        _config.eef("l_sole")->contact = contact_left;

        _init_pos = robot->skeleton()->getPositions().tail(robot->skeleton()->getNumDofs() - 6);

        _config.eef("root_link")->desired.pose.tail(3)[2] -= 0.15;
        _config.eef("root_link")->desired.pose.tail(3)[0] += 0.12;

        _config.eef("r_hand")->desired.pose.tail(3)[2] -= 0.1;
        _config.eef("r_hand")->desired.pose.tail(3)[0] += 0.02;
        _config.eef("l_hand")->desired.pose.tail(3)[2] -= 0.1;
        _config.eef("l_hand")->desired.pose.tail(3)[0] += 0.02;
    }

    Eigen::VectorXd calculate(double t) override
    {
        static double t0 = t;
        static bool f = true;
        double w = 100.;
        auto robot = _robot.lock();
        _config.skeleton()->setPositions(robot->skeleton()->getPositions());
        _config.skeleton()->setVelocities(robot->skeleton()->getVelocities());
        _config.update(true); // update contact information as well

        _solver->clear_all();
        Eigen::VectorXd target = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs() * 2 + 2 * 6);
        // target.segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs()).tail(_control_dof) = robot->skeleton()->getCoriolisAndGravityForces().tail(_control_dof);

        if ((t - t0 > 10.) && f) {
            _config.eef("root_link")->desired.pose.tail(3)[2] += 0.15;
            _config.eef("root_link")->desired.pose.tail(3)[0] -= 0.12;

            _config.eef("r_hand")->desired.pose.tail(3)[2] += 0.1;
            _config.eef("r_hand")->desired.pose.tail(3)[0] -= 0.02;
            _config.eef("l_hand")->desired.pose.tail(3)[2] += 0.1;
            _config.eef("l_hand")->desired.pose.tail(3)[0] -= 0.02;

            f = false;
        }

        // if (!f) {
        //     w = 10.;
        // }

        // whc::control::PDGains gains;
        // gains.kp = Eigen::VectorXd::Constant(6, 50.);
        // // gains.kp.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
        // gains.kd = Eigen::VectorXd::Constant(6, 200.);
        // // gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control

        // Add accelerations tasks for end-effectors
        for (size_t i = 0; i < _config.num_eefs(); i++) {
            whc::control::PDGains gains;
            gains.kp = Eigen::VectorXd::Constant(6, w);
            // gains.kp.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
            gains.kd = Eigen::VectorXd::Constant(6, w);
            // gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control

            auto eef = _config.eef(i);
            Eigen::VectorXd acc = whc::control::feedback(eef->state, eef->desired, gains);
            // acc = Eigen::VectorXd::Zero(6);

            Eigen::VectorXd weights = Eigen::VectorXd::Constant(6, 100.);
            // weights.head(3) = Eigen::VectorXd::Zero(3); // no orientation control

            // if this is a contact
            if (eef->keep_contact) {
                weights.array() *= 100.;
                // we want zero accelerations
                acc = Eigen::VectorXd::Zero(6);
                // add a contact constraint
                _solver->add_constraint(whc::utils::make_unique<whc::dyn::constraint::ContactConstraint>(_config.skeleton(), eef->body_name, eef->contact));
            }
            if (eef->body_name == "head") {
                weights.tail(3) = Eigen::VectorXd::Zero(3);
                acc.tail(3) = Eigen::VectorXd::Zero(3);
                // auto bd = _config.skeleton()->getBodyNode(eef->body_name);
                // Eigen::Matrix3d bd_trans = bd->getWorldTransform().linear();
                // acc.head(3) = bd_trans * acc.head(3);
            }
            if (eef->body_name == "chest") {
                weights.tail(3) = Eigen::VectorXd::Zero(3);
                acc.tail(3) = Eigen::VectorXd::Zero(3);
                // auto bd = _config.skeleton()->getBodyNode(eef->body_name);
                // Eigen::Matrix3d bd_trans = bd->getWorldTransform().linear();
                // acc.head(3) = bd_trans * acc.head(3);
                // gains.kp = Eigen::VectorXd::Constant(6, 10.);
                // gains.kp.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
                // gains.kd = Eigen::VectorXd::Constant(6, 10.);
                // gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
            }
            // if (eef->body_name == "root_link") {
            //     weights.head(3) = Eigen::VectorXd::Zero(3);
            //     acc.head(3) = Eigen::VectorXd::Zero(3);
            // }

            // if (eef->body_name != "head" && eef->body_name != "chest" && eef->body_name != "root_link")
            //     acc = Eigen::VectorXd::Zero(6);
            // if (eef->body_name == "r_hand") {
            // std::cout << eef->body_name << std::endl;
            // std::cout << "pose: " << eef->state.pose.transpose() << std::endl;
            // std::cout << "desired: " << eef->desired.pose.transpose() << std::endl;
            // Eigen::VectorXd pos_error = eef->desired.pose - eef->state.pose;
            // pos_error.head(3) = whc::utils::rotation_error(dart::math::expMapRot(eef->desired.pose.head(3)), dart::math::expMapRot(eef->state.pose.head(3)));
            // std::cout << "error: " << pos_error.transpose() << std::endl;
            // std::cout << "vel: " << eef->state.vel.transpose() << std::endl;
            // std::cout << "control: " << acc.transpose() << std::endl;
            // std::cout << "-------------------" << std::endl;
            //     // std::cin.get();
            // }
            _solver->add_task(whc::utils::make_unique<whc::dyn::task::AccelerationTask>(_config.skeleton(), eef->body_name, acc, weights));

            // // sanity checks
            // std::cout << "CHECKS" << std::endl;
            // auto bd = _config.skeleton()->getBodyNode(eef->body_name);

            // Eigen::MatrixXd jacobian = _config.skeleton()->getWorldJacobian(bd);

            // std::cout << (jacobian * _config.skeleton()->getVelocities()).transpose() << std::endl;
            // std::cout << eef->state.vel.transpose() << std::endl;
        }
        // std::cout << "-------------------" << std::endl;

        // Add COM acceleration task
        whc::utils::Frame state;
        state.pose = Eigen::VectorXd::Zero(6);
        state.vel = Eigen::VectorXd::Zero(6);
        state.acc = Eigen::VectorXd::Zero(6);
        whc::utils::ControlFrame desired = state;
        whc::control::PDGains gains;
        gains.kp = Eigen::VectorXd::Constant(6, 0.); // no position control
        gains.kd = Eigen::VectorXd::Constant(6, 100.);
        // gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
        state.vel = robot->skeleton()->getCOMSpatialVelocity();
        Eigen::VectorXd com_acc = whc::control::feedback(state, desired, gains);
        // std::cout << state.vel.transpose() << std::endl;
        // std::cout << com_acc.transpose() << std::endl;
        // std::cout << "----" << std::endl;
        // com_acc = Eigen::VectorXd::Zero(6);
        _solver->add_task(whc::utils::make_unique<whc::dyn::task::COMAccelerationTask>(robot->skeleton(), com_acc, 80.));

        // Add regularization task
        Eigen::VectorXd gweights = Eigen::VectorXd::Constant(target.size(), 0.01);
        gweights.tail(12) = Eigen::VectorXd::Ones(12);
        // This is important for stability
        gweights.head(robot->skeleton()->getNumDofs()) = Eigen::VectorXd::Constant(robot->skeleton()->getNumDofs(), 10.);
        // // gweights.tail(12) = Eigen::VectorXd::Constant(12, 0.1);
        // double Kp = 100.;
        // double Kd = 100.;
        // // Eigen::VectorXd posture = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs());
        // // Eigen::VectorXd upper = robot->skeleton()->getPositionUpperLimits().tail(robot->skeleton()->getNumDofs() - 6);
        // // Eigen::VectorXd lower = robot->skeleton()->getPositionLowerLimits().tail(robot->skeleton()->getNumDofs() - 6);
        // // posture.tail(robot->skeleton()->getNumDofs() - 6) = (upper - lower) / 2.;
        // // // std::cout << posture.transpose() << std::endl;
        // // target.head(robot->skeleton()->getNumDofs()) = Kp * (posture - robot->skeleton()->getPositions()) - Kd * robot->skeleton()->getVelocities();
        // // target.head(6) = Eigen::VectorXd::Zero(6);
        // target.head(robot->skeleton()->getNumDofs()).tail(_control_dof) = Kp * (_init_pos - robot->skeleton()->getPositions().tail(_control_dof)) - Kd * robot->skeleton()->getVelocities().tail(_control_dof);
        // // std::cout << "q: " << robot->skeleton()->getPositions().tail(_control_dof).transpose() << std::endl;
        // // std::cout << "q_d: " << _init_pos.transpose() << std::endl;

        _solver->add_task(whc::utils::make_unique<whc::dyn::task::DirectTrackingTask>(_config.skeleton(), target, gweights));

        // // Posture task
        // Eigen::VectorXd pweights = Eigen::VectorXd::Constant(robot->skeleton()->getNumDofs(), 10000.);
        // pweights.head(6) = Eigen::VectorXd::Zero(6);
        // _solver->add_task(whc::utils::make_unique<whc::dyn::task::PostureTask>(robot->skeleton(), _init_pos, pweights));

        // Add dynamics constraint
        _solver->add_constraint(whc::utils::make_unique<whc::dyn::constraint::DynamicsConstraint>(_config.skeleton()));
        // Add joint limits constraint
        _solver->add_constraint(whc::utils::make_unique<whc::dyn::constraint::JointLimitsConstraint>(_config.skeleton()));

        Eigen::VectorXd commands = _prev_tau.tail(_control_dof);
        if (_solver->solve()) {
            commands = _solver->solution().segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs()).tail(_control_dof);
            _prev_tau = _solver->solution().segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs());
        }

        // std::cout << "F: " << _solver->solution().tail(12).transpose() << std::endl;
        // std::cout << "qddot: " << _solver->solution().head(robot->skeleton()->getNumDofs()).transpose() << std::endl;

        // std::cin.get();

        return commands;
    }

    std::shared_ptr<robot_dart::control::RobotControl> clone() const
    {
        return std::make_shared<QPControl>(*this);
    }

protected:
    std::shared_ptr<whc::dyn::solver::IDSolver> _solver;
    Eigen::VectorXd _prev_tau, _init_pos;
    whc::control::Configuration _config;
};

void stabilize_robot(const std::shared_ptr<robot_dart::Robot>& robot, robot_dart::RobotDARTSimu& simu)
{
    robot->add_controller(std::make_shared<robot_dart::control::PDControl>(robot->skeleton()->getPositions().tail(robot->skeleton()->getNumDofs() - 6)));
    // std::static_pointer_cast<robot_dart::control::PDControl>(robot->controller(0))->set_pd(200., 10.);
    std::static_pointer_cast<robot_dart::control::PDControl>(robot->controller(0))->set_pd(1000., 10.);

    simu.run(3.);

    robot->clear_controllers();
}

int main()
{
    std::string model = "iCubNancy01";
    whc::icub_example::iCub icub(model, model);

    auto icub_robot = icub.robot();
    icub_robot->set_position_enforced(true);
    icub_robot->skeleton()->disableSelfCollisionCheck();
    icub_robot->skeleton()->setPosition(5, 0.625);
    if (model == "iCubNancy01")
        icub_robot->skeleton()->setPosition(5, 0.6 + 1e-5);
    // for (size_t i = 6; i < icub_robot->skeleton()->getNumDofs(); i++) {
    //     icub_robot->skeleton()->getDof(i)->getJoint()->setDampingCoefficient(0, 0.);
    //     icub_robot->skeleton()->getDof(i)->getJoint()->setCoulombFriction(0, 0.);
    // }

    Eigen::VectorXd lb, ub;
    lb = icub_robot->skeleton()->getForceLowerLimits();
    ub = icub_robot->skeleton()->getForceUpperLimits();

    lb = lb.unaryExpr([](double x) {if(x<-84.) return -84.; return x; });
    ub = ub.unaryExpr([](double x) {if(x>84.) return 84.; return x; });
    // lb.head(6) = Eigen::VectorXd::Zero(6);
    // ub.head(6) = Eigen::VectorXd::Zero(6);
    icub_robot->skeleton()->setForceLowerLimits(lb);
    icub_robot->skeleton()->setForceUpperLimits(ub);

    lb = icub_robot->skeleton()->getVelocityLowerLimits();
    ub = icub_robot->skeleton()->getVelocityLowerLimits();

    lb = lb.unaryExpr([](double x) {if(x<-100.) return -100.; return x; });
    ub = ub.unaryExpr([](double x) {if(x>100.) return 100.; return x; });
    // lb.head(6) = Eigen::VectorXd::Zero(6);
    // ub.head(6) = Eigen::VectorXd::Zero(6);
    icub_robot->skeleton()->setVelocityLowerLimits(lb);
    icub_robot->skeleton()->setVelocityLowerLimits(ub);

    icub_robot->skeleton()->getJoint("l_shoulder_roll")->setPosition(0, 0.2);
    icub_robot->skeleton()->getJoint("r_shoulder_roll")->setPosition(0, 0.2);

    icub_robot->skeleton()->getJoint("r_hip_pitch")->setPosition(0, 0.3);
    icub_robot->skeleton()->getJoint("l_hip_pitch")->setPosition(0, 0.3);

    icub_robot->skeleton()->getJoint("r_knee")->setPosition(0, -0.6);
    icub_robot->skeleton()->getJoint("l_knee")->setPosition(0, -0.6);

    icub_robot->skeleton()->getJoint("r_ankle_pitch")->setPosition(0, -0.3);
    icub_robot->skeleton()->getJoint("l_ankle_pitch")->setPosition(0, -0.3);

    // icub_robot->skeleton()->getJoint("torso_yaw")->setPosition(0, M_PI / 5.);
    // icub_robot->skeleton()->getJoint("torso_pitch")->setPosition(0, 0.5);
    // icub_robot->skeleton()->getJoint("torso_roll")->setPosition(0, 0.25);

    robot_dart::RobotDARTSimu simu(0.005);
    simu.set_collision_detector("fcl");
#ifdef GRAPHIC
    simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>(&simu));
    std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(simu.graphics())->look_at({0., 2., 1.5}, {0., 0., 0.5});
#endif
    simu.add_robot(icub_robot);
    simu.add_floor();

    // first stabilize the robot
    stabilize_robot(icub_robot, simu);

    std::cout << "Stabilized robot!" << std::endl;

    icub_robot->add_controller(std::make_shared<QPControl>());

    // simu.run(5.);
    // std::cout << "Applying force!" << std::endl;
    // icub_robot->skeleton()->getBodyNode("chest")->setExtForce({0., 20., 0.});
    // simu.run(0.1);
    // icub_robot->skeleton()->getBodyNode("chest")->clearExternalForces();

    simu.run(120.);

    return 0;
}
