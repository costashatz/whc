#include <iostream>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
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
        _solver->set_qp_solver<whc::qp_solver::QPOases>();
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

        _init_com[0] += 0.04;
        _init_com[2] -= 0.1;

        // _config.eef("root_link")->desired.pose.tail(3)[2] -= 0.15;
        // _config.eef("root_link")->desired.pose.tail(3)[2] -= 0.25;
        // _config.eef("root_link")->desired.pose.tail(3)[0] += 0.12;

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
        w(3) = 1.;
        w(4) = 1.;
        w(5) = 1.;
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
    simu.run(3.);
}

int main()
{
    std::string model = "iCubNancy01";
    whc::icub_example::iCub icub(model, model);

    auto icub_robot = icub.robot();
    icub_robot->set_actuator_types(dart::dynamics::Joint::SERVO);
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
    simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::FCLCollisionDetector::create());
#ifdef GRAPHIC
    simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
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
