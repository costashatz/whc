#include <iostream>
#include <robot_dart/control/pd_control.hpp>
#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#include "iCub.hpp"

#include <whc/solver/qp_oases.hpp>
#include <whc/solver/whc_solver.hpp>

#include <chrono>

class QPControl : public robot_dart::control::RobotControl {
public:
    QPControl() : robot_dart::control::RobotControl() {}

    void configure() override
    {
        _active = true;
        auto robot = _robot.lock();
        _solver = std::make_shared<whc::solver::WhcSolver>(robot);
        _solver->set_qp_solver<whc::solver::QPOases>();
        _prev_tau = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs());
    }

    Eigen::VectorXd calculate(double t) override
    {
        auto robot = _robot.lock();

        _solver->clear_all();
        Eigen::VectorXd target = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs() * 2 + 2 * 6);
        target.segment(robot->skeleton()->getNumDofs(), robot->skeleton()->getNumDofs()).tail(_control_dof) = robot->skeleton()->getCoriolisAndGravityForces().tail(_control_dof);
        double task_weight = 10000.;
        double gen_weight = 0.1;

        _solver->add_task(whc::utils::make_unique<whc::task::COMAccelerationTask>(robot->skeleton(), Eigen::VectorXd::Zero(6)), task_weight);
        _solver->add_task(whc::utils::make_unique<whc::task::AccelerationTask>(robot->skeleton(), "imu_frame", Eigen::VectorXd::Zero(6)), task_weight);
        _solver->add_task(whc::utils::make_unique<whc::task::AccelerationTask>(robot->skeleton(), "r_hand", Eigen::VectorXd::Zero(6)), task_weight);
        _solver->add_task(whc::utils::make_unique<whc::task::AccelerationTask>(robot->skeleton(), "l_hand", Eigen::VectorXd::Zero(6)), task_weight);
        // regularization
        _solver->add_task(whc::utils::make_unique<whc::task::DirectTrackingTask>(robot->skeleton(), target), gen_weight);
        _solver->add_constraint(whc::utils::make_unique<whc::constraint::DynamicsConstraint>(robot->skeleton()));
        // _solver->add_task(whc::utils::make_unique<whc::task::TauDiffTask>(robot->skeleton(), _prev_tau), gen_weight);

        _solver->add_constraint(whc::utils::make_unique<whc::constraint::JointLimitsConstraint>(robot->skeleton()));
        Eigen::VectorXd up(3), t1(3), t2(3);
        up << 0., 0., 1.;
        t1 << 1., 0., 0.;
        t2 << 0., 1., 0.;
        whc::constraint::Contact c;
        c.mu = 1.;
        c.muR = 1.;
        c.normal = up;
        c.t1 = t1;
        c.t2 = t2;
        c.min_force = 0.;
        c.max_force = 500.;
        c.min = Eigen::VectorXd::Zero(6);
        c.max = Eigen::VectorXd::Zero(6);
        c.max.tail(3) << c.max_force, c.max_force, c.max_force;
        double foot_size_x = 0.16;
        double foot_size_y = 0.072;
        double rsole_x = 0.0214502260596;
        double max_torque = 100.;
        c.calculate_torque = true;
        if (c.calculate_torque) {
            c.min.head(3) << -max_torque, -max_torque, -max_torque;
            c.max.head(3) << max_torque, max_torque, max_torque;
        }
        c.d_y_min = -foot_size_y / 2. + 0.02;
        c.d_y_max = foot_size_y / 2. - 0.02;
        c.d_x_min = -rsole_x + 0.02;
        c.d_x_max = foot_size_x - rsole_x - 0.02;

        _solver->add_contact(task_weight, "r_sole", c);
        _solver->add_contact(task_weight, "l_sole", c);

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
    Eigen::VectorXd _prev_tau;
};

int main()
{
    whc::icub_example::iCub icub("MyiCub");

    // arm->add_controller(std::make_shared<QPControl>());
    auto icub_robot = icub.robot();
    icub_robot->set_position_enforced(false);
    icub_robot->skeleton()->disableSelfCollisionCheck();
    icub_robot->skeleton()->setPosition(5, 0.625);
    for (size_t i = 6; i < icub_robot->skeleton()->getNumDofs(); i++) {
        icub_robot->skeleton()->getDof(i)->getJoint()->setDampingCoefficient(0, 0.);
        icub_robot->skeleton()->getDof(i)->getJoint()->setCoulombFriction(0, 0.);
    }

    icub_robot->add_controller(std::make_shared<QPControl>());

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
    simu.run(20.);

    return 0;
}
