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

#include <whc/control/configuration.hpp>
#include <whc/control/feedback.hpp>
#include <whc/dynamics/constraint/constraints.hpp>
#include <whc/dynamics/solver/id_solver.hpp>
#include <whc/dynamics/task/tasks.hpp>
#include <whc/qp_solver/qp_oases.hpp>
#include <whc/utils/math.hpp>

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
        _solver->set_qp_solver<whc::qp_solver::QPOases>();
        _config = whc::control::Configuration(skel);
        _config.add_eef("iiwa_link_ee", false); // no need for contacts

        _config.eef(0)->desired = _config.eef(0)->state;
        _config.eef(0)->desired.pose << -1.08292e-13, -1.0472, -4.56493e-12, -0.81926, 4.55583e-12, 0.833;
        _config.eef(0)->desired.vel = Eigen::VectorXd::Zero(6);
        _config.eef(0)->desired.acc = Eigen::VectorXd::Zero(6);
    }

    Eigen::VectorXd calculate(double) override
    {
        auto robot = _robot.lock();
        _solver->clear_all();

        _config.skeleton()->setPositions(robot->skeleton()->getPositions());
        _config.skeleton()->setVelocities(robot->skeleton()->getVelocities());
        _config.update(false); // no need for contacts

        Eigen::VectorXd target = Eigen::VectorXd::Zero(robot->skeleton()->getNumDofs() * 2);
        // target.tail(robot->skeleton()->getNumDofs()) = robot->skeleton()->getCoriolisAndGravityForces();

        Eigen::VectorXd weights = Eigen::VectorXd::Constant(6, 1.);
        // weights.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
        // weights.head(3) = Eigen::VectorXd::Constant(3, 50.); // smaller weights for orientation

        whc::control::PDGains gains;
        gains.kp = Eigen::VectorXd::Constant(6, 5.);
        // gains.kp.head(3) = Eigen::VectorXd::Zero(3); // no orientation control
        gains.kd = Eigen::VectorXd::Constant(6, 5.);
        // gains.kd.head(3) = Eigen::VectorXd::Zero(3); // no orientation control

        auto eef = _config.eef(0);
        Eigen::VectorXd acc = whc::control::feedback(eef->state, eef->desired, gains);
        // std::cout << "pose: " << eef->state.pose.transpose() << std::endl;
        // std::cout << "desired: " << eef->desired.pose.transpose() << std::endl;
        // Eigen::VectorXd pos_error = eef->desired.pose - eef->state.pose;
        // pos_error.head(3) = whc::utils::rotation_error(dart::math::expMapRot(eef->desired.pose.head(3)), dart::math::expMapRot(eef->state.pose.head(3)));
        // std::cout << "error: " << pos_error.transpose() << std::endl;
        // std::cout << "vel: " << eef->state.vel.transpose() << std::endl;
        // std::cout << "control: " << acc.transpose() << std::endl;
        // std::cout << "-------------------" << std::endl;
        _solver->add_task(whc::utils::make_unique<whc::dyn::task::AccelerationTask>(_config.skeleton(), "iiwa_link_ee", acc, weights));

        Eigen::VectorXd gweights = Eigen::VectorXd::Constant(target.size(), 0.001);
        // This is important for stability
        gweights.head(robot->skeleton()->getNumDofs()) = Eigen::VectorXd::Constant(robot->skeleton()->getNumDofs(), 0.1);
        // double Kp = 5.;
        // double Kd = 5.;
        // target.head(robot->skeleton()->getNumDofs()) = -Kp * robot->skeleton()->getPositions() - Kd * robot->skeleton()->getVelocities();

        _solver->add_task(whc::utils::make_unique<whc::dyn::task::DirectTrackingTask>(_config.skeleton(), target, gweights));

        _solver->add_constraint(whc::utils::make_unique<whc::dyn::constraint::DynamicsConstraint>(_config.skeleton(), false));
        // _solver->add_constraint(whc::utils::make_unique<whc::dyn::constraint::JointLimitsConstraint>(_config.skeleton()));
        _solver->solve();

        Eigen::VectorXd commands = _solver->solution().tail(robot->skeleton()->getNumDofs());
        return commands;
    }

    std::shared_ptr<robot_dart::control::RobotControl> clone() const override
    {
        return std::make_shared<QPControl>(*this);
    }

protected:
    std::shared_ptr<whc::dyn::solver::IDSolver> _solver;
    whc::control::Configuration _config;
};

int main()
{
    std::vector<std::pair<std::string, std::string>> packages = {{"iiwa_description", std::string(RESPATH)}};
    auto arm = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/iiwa14.urdf", packages);
    arm->fix_to_world();
    arm->set_position_enforced(true);
    arm->skeleton()->disableSelfCollisionCheck();
    arm->skeleton()->setPosition(1, M_PI / 3.);

    // for (size_t i = 0; i < arm->skeleton()->getNumDofs(); i++) {
    //     arm->skeleton()->getDof(i)->getJoint()->setDampingCoefficient(0, 0.);
    //     arm->skeleton()->getDof(i)->getJoint()->setCoulombFriction(0, 0.);
    // }

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