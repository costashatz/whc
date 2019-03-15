#include <iostream>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include <whc/control/configuration.hpp>
#include <whc/control/feedback.hpp>
#include <whc/solver/qp_oases.hpp>
#include <whc/solver/whc_solver.hpp>
#include <whc/utils/math.hpp>

#include "iCub.hpp"

class QPControl : public robot_dart::control::RobotControl {
public:
    QPControl() : robot_dart::control::RobotControl() {}

    void configure() override
    {
        _active = true;
        auto robot = _robot.lock();
        _solver = std::make_shared<whc::solver::WhcSolver>(robot->skeleton());
        _solver->set_qp_solver<whc::solver::QPOases>();
        _prev_tau = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs());
        _init_pos = robot->skeleton()->getPositions();

        _config = whc::control::Configuration(robot->skeleton());
        _config.add_eef("imu_frame", false); // no need for contacts for head
        _config.add_eef("root_link", false); // no need for contacts for root link
        _config.add_eef("r_hand", false); // no need for contacts for hands
        _config.add_eef("l_hand", false);

        _config.add_eef("r_sole"); // contacts for feet
        _config.add_eef("l_sole");

        // Set desired state -- keep end-effectors posture
        for (size_t i = 0; i < _config.num_eefs(); i++) {
            _config.eef(i)->desired = _config.eef(i)->state;
            _config.eef(i)->desired.vel = Eigen::VectorXd::Zero(6);
            _config.eef(i)->desired.acc = Eigen::VectorXd::Zero(6);
        }

        // Contact information for feet eefs
        whc::utils::Contact contact_right;
        contact_right.mu = 1.;
        contact_right.muR = 1.;
        // contact_right.nz = cfg._r_nz;
        // contact_right.nx = cfg._r_nx;
        // contact_right.ny = cfg._r_ny;
        contact_right.min_force = 0.;
        contact_right.max_force = 500.;
        contact_right.min = Eigen::VectorXd::Zero(6);
        contact_right.max = Eigen::VectorXd::Zero(6);
        contact_right.max.tail(3) << contact_right.max_force, contact_right.max_force, contact_right.max_force;
        double max_torque = 100.;
        contact_right.calculate_torque = true;
        if (contact_right.calculate_torque) {
            contact_right.min.head(3) << -max_torque, -max_torque, -max_torque;
            contact_right.max.head(3) << max_torque, max_torque, max_torque;
        }

        // iCub Nancy
        double foot_size_x = 0.12;
        double foot_size_y = 0.04;
        double sole_x = 0.0214502260596;
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
        // contact_left.nz = cfg._l_nz;
        // contact_left.nx = cfg._l_nx;
        // contact_left.ny = cfg._l_ny;

        contact_left.d_y_max = contact_right.d_y_max + sole_y;
        contact_left.d_y_min = contact_right.d_y_min + sole_y;

        _config.eef("r_sole")->keep_contact = true;
        _config.eef("r_sole")->contact = contact_right;
        _config.eef("l_sole")->keep_contact = true;
        _config.eef("l_sole")->contact = contact_left;
    }

    Eigen::VectorXd calculate(double t) override
    {
        auto robot = _robot.lock();
        _config.update(true); // update contact information as well

        _solver->clear_all();
        Eigen::VectorXd target = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs() * 2 + 2 * 6);
        target.segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs()).tail(_control_dof) = robot->skeleton()->getCoriolisAndGravityForces().tail(_control_dof);

        Eigen::VectorXd weights = Eigen::VectorXd::Constant(6, 10000.);
        // weights.head(3) = Eigen::VectorXd::Zero(3); // no orientation control

        whc::control::PDGains gains;
        gains.kp = Eigen::VectorXd::Constant(6, 10.);
        // gains.kp.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
        gains.kd = Eigen::VectorXd::Constant(6, 0.3);
        // gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control

        // Add accelerations tasks for end-effectors
        for (size_t i = 0; i < _config.num_eefs(); i++) {
            auto eef = _config.eef(i);
            Eigen::VectorXd acc = whc::control::feedback(eef->state, eef->desired, gains);
            // acc = Eigen::VectorXd::Zero(6);

            // if this is a contact
            if (eef->keep_contact) {
                // we want zero accelerations
                acc = Eigen::VectorXd::Zero(6);
                // add a contact constraint
                _solver->add_constraint(whc::utils::make_unique<whc::constraint::ContactConstraint>(_config.skeleton(), eef->body_name, eef->contact));
            }
            // if (eef->body_name == "r_hand") {
            //     std::cout << "pose: " << eef->state.pose.transpose() << std::endl;
            //     std::cout << "desired: " << eef->desired.pose.transpose() << std::endl;
            //     Eigen::VectorXd pos_error = eef->desired.pose - eef->state.pose;
            //     pos_error.head(3) = whc::utils::rotation_error(dart::math::expMapRot(eef->desired.pose.head(3)), dart::math::expMapRot(eef->state.pose.head(3)));
            //     std::cout << "error: " << pos_error.transpose() << std::endl;
            //     std::cout << "vel: " << eef->state.vel.transpose() << std::endl;
            //     std::cout << "control: " << acc.transpose() << std::endl;
            //     std::cout << "-------------------" << std::endl;
            //     // std::cin.get();
            // }
            _solver->add_task(whc::utils::make_unique<whc::task::AccelerationTask>(_config.skeleton(), eef->body_name, acc, weights));
        }

        // Add COM acceleration task
        whc::utils::Frame state;
        state.pose = Eigen::VectorXd::Zero(6);
        state.vel = Eigen::VectorXd::Zero(6);
        state.acc = Eigen::VectorXd::Zero(6);
        whc::utils::ControlFrame desired = state;
        gains.kp = Eigen::VectorXd::Constant(6, 0.); // no position control
        gains.kd = Eigen::VectorXd::Constant(6, 1.);
        gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
        state.vel = robot->skeleton()->getCOMSpatialVelocity();
        Eigen::VectorXd com_acc = whc::control::feedback(state, desired, gains);
        _solver->add_task(whc::utils::make_unique<whc::task::COMAccelerationTask>(robot->skeleton(), com_acc, 10000.));

        // Add regularization task
        Eigen::VectorXd gweights = Eigen::VectorXd::Constant(target.size(), 0.001);
        // This is important for stability
        gweights.head(robot->skeleton()->getNumDofs()) = Eigen::VectorXd::Constant(robot->skeleton()->getNumDofs(), 100.);
        _solver->add_task(whc::utils::make_unique<whc::task::DirectTrackingTask>(robot->skeleton(), target, gweights));

        // // Posture task
        // Eigen::VectorXd pweights = Eigen::VectorXd::Constant(robot->skeleton()->getNumDofs(), 10000.);
        // pweights.head(6) = Eigen::VectorXd::Zero(6);
        // _solver->add_task(whc::utils::make_unique<whc::task::PostureTask>(robot->skeleton(), _init_pos, pweights));

        // Add dynamics constraint
        _solver->add_constraint(whc::utils::make_unique<whc::constraint::DynamicsConstraint>(robot->skeleton()));
        // Add joint limits constraint
        _solver->add_constraint(whc::utils::make_unique<whc::constraint::JointLimitsConstraint>(robot->skeleton()));

        _solver->solve();

        Eigen::VectorXd commands = _solver->solution().segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs()).tail(_control_dof);
        _prev_tau = _solver->solution().segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs());

        return commands;
    }

    std::shared_ptr<robot_dart::control::RobotControl> clone() const
    {
        return std::make_shared<QPControl>(*this);
    }

protected:
    std::shared_ptr<whc::solver::WhcSolver> _solver;
    Eigen::VectorXd _prev_tau, _init_pos;
    whc::control::Configuration _config;
};

void stabilize_robot(const std::shared_ptr<robot_dart::Robot>& robot, robot_dart::RobotDARTSimu& simu)
{
    std::vector<double> target(robot->skeleton()->getNumDofs() - 6);

    Eigen::VectorXd::Map(target.data(), target.size()) = robot->skeleton()->getPositions().tail(robot->skeleton()->getNumDofs() - 6);

    robot->add_controller(std::make_shared<robot_dart::control::PDControl>(target));
    // std::static_pointer_cast<robot_dart::control::PDControl>(robot->controller(0))->set_pd(1000., 10.);
    // std::static_pointer_cast<robot_dart::control::PDControl>(robot->controller(0))->set_pd(200., 10.);
    std::static_pointer_cast<robot_dart::control::PDControl>(robot->controller(0))->set_pd(1000., 10.);

    simu.run(10.);

    robot->clear_controllers();
}

int main()
{
    std::string model = "iCubNancy01";
    whc::icub_example::iCub icub(model, model);

    auto icub_robot = icub.robot();
    icub_robot->set_position_enforced(false);
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
    lb.head(6) = Eigen::VectorXd::Zero(6);
    ub.head(6) = Eigen::VectorXd::Zero(6);
    icub_robot->skeleton()->setForceLowerLimits(lb);
    icub_robot->skeleton()->setForceUpperLimits(ub);

    lb = icub_robot->skeleton()->getVelocityLowerLimits();
    ub = icub_robot->skeleton()->getVelocityLowerLimits();

    lb = lb.unaryExpr([](double x) {if(x<-100.) return -100.; return x; });
    ub = ub.unaryExpr([](double x) {if(x>100.) return 100.; return x; });
    lb.head(6) = Eigen::VectorXd::Zero(6);
    ub.head(6) = Eigen::VectorXd::Zero(6);
    icub_robot->skeleton()->setVelocityLowerLimits(lb);
    icub_robot->skeleton()->setVelocityLowerLimits(ub);

    robot_dart::RobotDARTSimu simu(0.005);
    simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
#ifdef GRAPHIC
    simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
#endif
    simu.add_robot(icub_robot);
    simu.add_floor();

    // first stabilize the robot
    stabilize_robot(icub_robot, simu);

    std::cout << "Stabilized robot!" << std::endl;

    icub_robot->add_controller(std::make_shared<QPControl>());

    simu.run(20.);

    return 0;
}
