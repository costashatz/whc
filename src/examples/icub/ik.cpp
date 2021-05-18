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
#include <whc/kinematics/constraint/constraints.hpp>
#include <whc/kinematics/solver/ik_solver.hpp>
#include <whc/kinematics/task/tasks.hpp>
#include <whc/qp_solver/qp_oases.hpp>
#ifdef USE_OSQP
#include <whc/qp_solver/osqp.hpp>
#endif
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
        _solver = std::make_shared<whc::kin::solver::IKSolver>(skel);
#ifdef USE_OSQP
        _solver->set_qp_solver<whc::qp_solver::OSQP>(200, false);
#else
        _solver->set_qp_solver<whc::qp_solver::QPOases>(0.005, 200, false);
#endif
        _prev_vel = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs());
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

        _init_pos = robot->skeleton()->getPositions().tail(robot->skeleton()->getNumDofs() - 6);
        _init_com = robot->skeleton()->getCOM();

        // _config.eef("root_link")->desired.pose.tail(3)[2] -= 0.15;
        // _config.eef("root_link")->desired.pose.tail(3)[2] -= 0.25;
        // _config.eef("root_link")->desired.pose.tail(3)[0] += 0.12;

        _init_com[0] += 0.04;
        _init_com[2] -= 0.1;

        _config.eef("r_hand")->desired.pose.tail(3)[2] -= 0.1;
        _config.eef("r_hand")->desired.pose.tail(3)[0] += 0.02;
        _config.eef("l_hand")->desired.pose.tail(3)[2] -= 0.1;
        _config.eef("l_hand")->desired.pose.tail(3)[0] += 0.02;
    }

    Eigen::VectorXd calculate(double t) override
    {
        static double t0 = t;
        static bool f = true;
        auto robot = _robot.lock();
        _config.skeleton()->setPositions(robot->skeleton()->getPositions());
        _config.skeleton()->setVelocities(robot->skeleton()->getVelocities());
        _config.update(true); // update contact information as well

        _solver->clear_all();

        if ((t - t0 > 10.) && f) {
            // _config.eef("root_link")->desired.pose.tail(3)[2] += 0.15;
            // _config.eef("root_link")->desired.pose.tail(3)[0] -= 0.12;

            _init_com[0] -= 0.04;
            _init_com[2] += 0.1;

            _config.eef("r_hand")->desired.pose.tail(3)[2] += 0.1;
            _config.eef("r_hand")->desired.pose.tail(3)[0] -= 0.02;
            _config.eef("l_hand")->desired.pose.tail(3)[2] += 0.1;
            _config.eef("l_hand")->desired.pose.tail(3)[0] -= 0.02;

            f = false;
        }

        // Add velocity tasks for end-effectors
        for (size_t i = 0; i < _config.num_eefs(); i++) {
            auto eef = _config.eef(i);

            Eigen::VectorXd pos_error = eef->desired.pose - eef->state.pose;
            pos_error.head(3) = whc::utils::rotation_error(dart::math::expMapRot(eef->desired.pose.head(3)), dart::math::expMapRot(eef->state.pose.head(3)));

            Eigen::VectorXd w = Eigen::VectorXd::Constant(6, 1.);
            if (eef->body_name == "chest" || eef->body_name == "head")
                w.tail(3).setZero();
            if (eef->body_name == "root_link") // since we are controlling the COM positioning, we ignore the position of the root_link
                w.tail(3).setZero();

            _solver->add_task(whc::utils::make_unique<whc::kin::task::VelocityTask>(_config.skeleton(), eef->body_name, pos_error, w));
        }

        // COM Velocity task
        Eigen::VectorXd pos_error = Eigen::VectorXd::Zero(6);
        pos_error.tail(3) = _init_com - _config.skeleton()->getCOM();
        Eigen::VectorXd w = Eigen::VectorXd::Zero(6);
        w.tail(3).setOnes();
        _solver->add_task(whc::utils::make_unique<whc::kin::task::COMVelocityTask>(_config.skeleton(), pos_error, w));

        // Add regularization task
        Eigen::VectorXd target = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs());
        Eigen::VectorXd gweights = Eigen::VectorXd::Constant(target.size(), 0.025);

        _solver->add_task(whc::utils::make_unique<whc::kin::task::DirectTrackingTask>(_config.skeleton(), target, gweights));

        // Add joint limits constraint
        _solver->add_constraint(whc::utils::make_unique<whc::kin::constraint::JointLimitsConstraint>(_config.skeleton()));

        _solver->solve();

        Eigen::VectorXd commands = _solver->solution().tail(_control_dof);

        // return 100. * (commands - _config.skeleton()->getVelocities().tail(_control_dof));
        return commands;
        // Eigen::VectorXd next_pos = _config.skeleton()->getPositions().tail(_control_dof) + commands * _config.skeleton()->getTimeStep();
        // Eigen::VectorXd curr_pos = _config.skeleton()->getPositions().tail(_control_dof); //+ _config.skeleton()->getVelocities().tail(_control_dof) * _config.skeleton()->getTimeStep();
        // // std::cout << (next_pos - curr_pos).norm() << std::endl;
        // Eigen::VectorXd c = 100000. * (next_pos - curr_pos) - 10. * _config.skeleton()->getVelocities().tail(_control_dof);

        // return c;
    }

    std::shared_ptr<robot_dart::control::RobotControl> clone() const
    {
        return std::make_shared<QPControl>(*this);
    }

protected:
    std::shared_ptr<whc::kin::solver::IKSolver> _solver;
    Eigen::VectorXd _prev_vel, _init_pos, _init_com;
    whc::control::Configuration _config;
};

void stabilize_robot(const std::shared_ptr<robot_dart::Robot>& robot, robot_dart::RobotDARTSimu& simu)
{
    // std::vector<double> target(robot->skeleton()->getNumDofs() - 6);

    // Eigen::VectorXd::Map(target.data(), target.size()) = robot->skeleton()->getPositions().tail(robot->skeleton()->getNumDofs() - 6);

    // robot->add_controller(std::make_shared<robot_dart::control::PDControl>(target));
    // std::static_pointer_cast<robot_dart::control::PDControl>(robot->controller(0))->set_pd(1000., 10.);

    simu.run(3.);

    robot->clear_controllers();
}

struct RecordDesc : public robot_dart::descriptor::BaseDescriptor {
public:
    RecordDesc(size_t desc_dump = 1) : robot_dart::descriptor::BaseDescriptor(desc_dump), _index(0) {}

    void operator()()
    {
        // if (_simu.robots().size() > 0) {
        //     auto robot = _simu.robots()[0];

        //     robot->skeleton()->clearExternalForces();
        //     if (_index % 400 == 0) { // every 2 seconds
        //         _count = 0;
        //         _f = Eigen::Vector3d::Random(); //.array() * 50.;
        //         // _f[2] = 0.;
        //         // _f[0] = 0.;
        //         // _f[1] = 0.;
        //         _f = _f.normalized().array() * 50.;
        //         // _f << 0., 1., 0.;
        //         // if (_index > 0) {
        //         //     _f << 0., -1., 0.;
        //         // }
        //         // robot->skeleton()->getBodyNode("chest")->setExtForce(_f);
        //     }
        //     if (_count < 50) {
        //         robot->skeleton()->getBodyNode("chest")->setExtForce(_f);
        //         std::cout << _simu.world()->getTime() << ": " << _f.transpose() << std::endl;
        //     }
        //     _count++;
        //     _index++;
        // }
    }

protected:
    int _index;
    int _count;
    Eigen::VectorXd _f;
};

int main()
{
    std::string model = "iCubNancy01";
    whc::icub_example::iCub icub(model, model);

    auto icub_robot = icub.robot();
    icub_robot->set_actuator_types("servo");
    icub_robot->set_position_enforced(false);
    // icub_robot->skeleton()->getRootJoint()->setPositionLimitEnforced(false);
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
    simu.set_graphics(std::make_shared<robot_dart::gui::magnum::Graphics>());
    std::static_pointer_cast<robot_dart::gui::magnum::Graphics>(simu.graphics())->look_at({0., 2., 1.5}, {0., 0., 0.5});
#endif
    simu.add_robot(icub_robot);
    simu.add_floor();

    // first stabilize the robot
    stabilize_robot(icub_robot, simu);

    std::cout << "Stabilized robot!" << std::endl;

    icub_robot->add_controller(std::make_shared<QPControl>());

    simu.add_descriptor(std::make_shared<RecordDesc>());

    // simu.run(5.);
    // std::cout << "Applying force!" << std::endl;
    // icub_robot->skeleton()->getBodyNode("chest")->setExtForce({0., 20., 0.});
    // simu.run(0.1);
    // icub_robot->skeleton()->getBodyNode("chest")->clearExternalForces();

    simu.run(120.);

    return 0;
}
