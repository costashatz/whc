#include <iostream>
#include <robot_dart/robot_dart_simu.hpp>

#include <dart/collision/bullet/BulletCollisionDetector.hpp>
#include <dart/constraint/ConstraintSolver.hpp>

#ifdef GRAPHIC
#include <robot_dart/graphics/graphics.hpp>
#endif

#include <icub/model/iCub.hpp>

#include <icub/solver/qp_solver.hpp>

int main()
{
    // std::vector<std::pair<std::string, std::string>> packages = {{"iiwa_description", std::string(RESPATH) + "/iiwa/"}};
    // auto arm = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/iiwa/iiwa14.urdf", packages);
    // // auto arm = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/arm.urdf");
    // arm->fix_to_world();

    // icub::solver::QPSolver qp(arm);
    // // qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(arm->skeleton(), "arm_link_5", Eigen::VectorXd::Zero(6)), 20.);
    // qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(arm->skeleton(), "iiwa_link_ee", Eigen::VectorXd::Zero(6)), 20.);
    // qp.add_task(icub::task::create_task<icub::task::DirectTrackingTask>(arm->skeleton(), Eigen::VectorXd::Zero(arm->skeleton()->getNumDofs() * 2)));
    // qp.add_constraint(icub::constraint::create_constraint<icub::constraint::DynamicsConstraint>(arm->skeleton(), false));
    // qp.solve();
    // std::srand(std::time(NULL));
    icub::model::iCub icub("MyiCub");

    icub::solver::QPSolver qp(icub.robot());
    double task_weight = 10000.;
    double gen_weight = 0.1;
    qp.add_task(icub::task::create_task<icub::task::COMAccelerationTask>(icub.skeleton(), Eigen::VectorXd::Zero(6)), task_weight);
    qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(icub.skeleton(), "r_hand", Eigen::VectorXd::Zero(6)), task_weight);
    qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(icub.skeleton(), "l_hand", Eigen::VectorXd::Zero(6)), task_weight);
    // regularization
    qp.add_task(icub::task::create_task<icub::task::DirectTrackingTask>(icub.skeleton(), Eigen::VectorXd::Zero(icub.skeleton()->getNumDofs() * 2 + 2 * 6)), gen_weight);
    qp.add_constraint(icub::constraint::create_constraint<icub::constraint::DynamicsConstraint>(icub.skeleton()));
    Eigen::VectorXd up(3), t1(3), t2(3);
    up << 0., 0., 1.;
    t1 << 1., 0., 0.;
    t2 << 0., 1., 0.;
    icub::constraint::Contact c;
    c.mu = 1.;
    c.normal = up;
    c.t1 = t1;
    c.t2 = t2;
    c.min_force = 0.;
    c.max_force = 500.;
    c.min = Eigen::VectorXd::Zero(6);
    c.max = Eigen::VectorXd::Zero(6);
    c.max.tail(3) << c.max_force, c.max_force, c.max_force;

    qp.add_contact(task_weight, "r_sole", c);
    qp.add_contact(task_weight, "l_sole", c);
    // qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(icub.skeleton(), "r_sole", Eigen::VectorXd::Zero(6)));
    // qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(icub.skeleton(), "l_sole", Eigen::VectorXd::Zero(6)));
    // qp.solve(Eigen::VectorXd::Zero(5 * 6));
    qp.solve();

    //     auto iCub_robot = icub.robot();
    //     iCub_robot->free_from_world();
    //     iCub_robot->set_position_enforced(true);
    //     iCub_robot->skeleton()->setPosition(5, 0.8);

    //     iCub_robot->set_actuator_types(dart::dynamics::Joint::SERVO);
    //     for (int i = 0; i < 6; i++)
    //         iCub_robot->skeleton()->getDof(i)->getJoint()->setActuatorType(dart::dynamics::Joint::FORCE);

    //     robot_dart::RobotDARTSimu simu(0.001);

    //     simu.world()->getConstraintSolver()->setCollisionDetector(dart::collision::BulletCollisionDetector::create());
    // #ifdef GRAPHIC
    //     simu.set_graphics(std::make_shared<robot_dart::graphics::Graphics>(simu.world()));
    // #endif
    //     simu.add_robot(iCub_robot);
    //     simu.add_floor(10., 0.2);

    //     simu.run(5);

    // /* Setup data of first QP. */
    // qpOASES::real_t H[2 * 2] = {1.0, 0.0, 0.0, 0.5};
    // qpOASES::real_t A[1 * 2] = {1.0, 1.0};
    // qpOASES::real_t g[2] = {1.5, 1.0};
    // qpOASES::real_t lb[2] = {0.5, -2.0};
    // qpOASES::real_t ub[2] = {5.0, 2.0};
    // qpOASES::real_t lbA[1] = {-1.0};
    // qpOASES::real_t ubA[1] = {2.0};
    // /* Setup data of second QP. */
    // qpOASES::real_t g_new[2] = {1.0, 1.5};
    // qpOASES::real_t lb_new[2] = {0.0, -1.0};
    // qpOASES::real_t ub_new[2] = {5.0, -0.5};
    // qpOASES::real_t lbA_new[1] = {-2.0};
    // qpOASES::real_t ubA_new[1] = {1.0};
    // /* Setting up QProblem object. */
    // std::cout << "Starting example" << std::endl;
    // qpOASES::QProblem example(2, 1);
    // auto options = example.getOptions();
    // options.printLevel = qpOASES::PL_LOW;
    // example.setOptions(options);
    // /* Solve first QP. */
    // int nWSR = 10;
    // example.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    // std::cout << "Finished example" << std::endl;

    return 0;
}
