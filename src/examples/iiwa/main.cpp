#include <iostream>
#include <robot_dart/control/robot_control.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif

#include <dart/collision/CollisionObject.hpp>
#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>

#include <whc/solver/qp_oases.hpp>
#include <whc/solver/whc_solver.hpp>

class QPControl : public robot_dart::control::RobotControl {
public:
    QPControl() : robot_dart::control::RobotControl() {}

    void configure() override
    {
        _active = true;
        auto robot = _robot.lock();
        _solver = std::make_shared<whc::solver::WhcSolver>(robot);
        _solver->set_qp_solver<whc::solver::QPOases>();
    }

    Eigen::VectorXd calculate(double) override
    {
        auto robot = _robot.lock();
        _solver->clear_all();
        Eigen::VectorXd target = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs() * 2);
        target.tail(robot->skeleton()->getNumDofs()) = robot->skeleton()->getCoriolisAndGravityForces();
        _solver->add_task(whc::utils::make_unique<whc::task::AccelerationTask>(robot->skeleton(), "iiwa_link_ee", Eigen::VectorXd::Zero(6)), 10000.);
        _solver->add_task(whc::utils::make_unique<whc::task::DirectTrackingTask>(robot->skeleton(), target), 0.001);
        _solver->add_constraint(whc::utils::make_unique<whc::constraint::DynamicsConstraint>(robot->skeleton(), false));
        _solver->add_constraint(whc::utils::make_unique<whc::constraint::JointLimitsConstraint>(robot->skeleton()));
        _solver->solve();

        Eigen::VectorXd commands = _solver->solution().tail(robot->skeleton()->getNumDofs());
        return commands;
    }

    std::shared_ptr<robot_dart::control::RobotControl> clone() const
    {
        return std::make_shared<QPControl>(*this);
    }

protected:
    std::shared_ptr<whc::solver::WhcSolver> _solver;
};

int main()
{
    std::vector<std::pair<std::string, std::string>> packages = {{"iiwa_description", std::string(RESPATH)}};
    auto arm = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/iiwa14.urdf", packages);
    arm->fix_to_world();
    arm->set_position_enforced(false);
    arm->skeleton()->disableSelfCollisionCheck();
    arm->skeleton()->setPosition(1, M_PI / 3.);

    for (size_t i = 0; i < arm->skeleton()->getNumDofs(); i++) {
        arm->skeleton()->getDof(i)->getJoint()->setDampingCoefficient(0, 0.);
        arm->skeleton()->getDof(i)->getJoint()->setCoulombFriction(0, 0.);
    }

    arm->add_controller(std::make_shared<QPControl>());

    robot_dart::RobotDARTSimu simu(0.005);
    simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
#ifdef GRAPHIC
    simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
#endif
    simu.add_robot(arm);
    simu.add_floor();
    simu.run(20.);
}