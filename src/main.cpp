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
    std::srand(std::time(NULL));
    icub::model::iCub icub("MyiCub");

    icub::solver::QPSolver qp(&icub);
    qp.add_task(icub::task::create_task<icub::task::COMAccelerationTask>(icub.skeleton(), Eigen::VectorXd::Zero(6)));
    qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(icub.skeleton(), "r_hand", Eigen::VectorXd::Zero(6)));
    qp.add_task(icub::task::create_task<icub::task::AccelerationTask>(icub.skeleton(), "l_hand", Eigen::VectorXd::Zero(6)));
    qp.add_constraint(icub::constraint::create_constraint<icub::constraint::DynamicsConstraint>(icub.skeleton()));
    qp.add_contact("r_sole", 0.1);
    qp.add_contact("l_sole", 0.1);
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
