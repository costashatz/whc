#ifndef ICUB_MODEL_ICUB_HPP
#define ICUB_MODEL_ICUB_HPP

#include <icub/model/iCub_common.hpp>

#include <robot_dart/robot.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/EndEffector.hpp>
#include <dart/dynamics/Joint.hpp>

namespace icub {
    namespace model {
        class iCub {
        public:
            iCub(const std::string& name, const std::string& model = "iCubGazeboV2_5_plus")
            {
                _robot = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/robots/" + model + "/model.urdf", packages(), name);
                _fix_masses();
                _create_end_effectors();

                // std::cout << _r_hand->getWorldJacobian() << std::endl
                //           << std::endl;
                // std::cout << _r_hand->getJacobian(_robot->skeleton()->getBodyNode(base_link())) << std::endl
                //           << std::endl;
                // std::cout << _robot->skeleton()->getCOMJacobian().rows() << "x" << _robot->skeleton()->getCOMJacobian().cols() << std::endl
                //           << std::endl;
                // std::cout << _robot->skeleton()->getRootJoint()->getRelativeTransform().matrix() << std::endl;
            }

            // Basic helpers
            std::shared_ptr<robot_dart::Robot> robot()
            {
                return _robot;
            }

            dart::dynamics::SkeletonPtr skeleton()
            {
                return _robot->skeleton();
            }

            dart::dynamics::EndEffector* r_hand()
            {
                return _r_hand;
            }

            dart::dynamics::EndEffector* l_hand()
            {
                return _l_hand;
            }

            dart::dynamics::EndEffector* r_sole()
            {
                return _r_sole;
            }

            dart::dynamics::EndEffector* l_sole()
            {
                return _l_sole;
            }

            dart::dynamics::EndEffector* head()
            {
                return _head;
            }

        protected:
            std::shared_ptr<robot_dart::Robot> _robot;
            dart::dynamics::EndEffector *_r_hand, *_l_hand, *_r_sole, *_l_sole, *_head;

            void _fix_masses()
            {
                for (size_t i = 0; i < _robot->skeleton()->getNumBodyNodes(); i++) {
                    auto bd = _robot->skeleton()->getBodyNode(i);
                    double ixx, ixy, ixz, iyy, iyz, izz;
                    bd->getMomentOfInertia(ixx, iyy, izz, ixy, ixz, iyz);
                    bool dummy_obj = (ixx == 1.) && (iyy == 1.) && (izz == 1.) && (ixy == 0.) && (ixz == 0.) && (iyz == 0.);

                    if (bd->getMass() == 1. && dummy_obj) {
                        bd->setMass(1e-8);
                    }
                }
            }

            void _create_end_effectors()
            {
                auto legs = leg_eefs();
                auto hands = arm_eefs();

                // std::cout << _robot->skeleton()->getBodyNode(hands[0])->getWorldJacobian().rows() << "x" << _robot->skeleton()->getBodyNode(hands[0])->getWorldJacobian().cols() << std::endl;
                // Eigen::MatrixXd jac = _robot->skeleton()->getWorldJacobian(_robot->skeleton()->getBodyNode(hands[0]));
                // // std::cout << jac.rows() << "x" << jac.cols() << std::endl;
                // std::cout << jac << std::endl
                //           << std::endl;
                // std::cout << _robot->skeleton()->getBodyNode(hands[0])->getWorldJacobian() << std::endl
                //           << std::endl;

                _r_hand = _robot->skeleton()->getBodyNode(hands[0])->createEndEffector(hands[0]);
                _l_hand = _robot->skeleton()->getBodyNode(hands[1])->createEndEffector(hands[1]);

                _r_sole = _robot->skeleton()->getBodyNode(legs[0])->createEndEffector(legs[0]);
                _l_sole = _robot->skeleton()->getBodyNode(legs[1])->createEndEffector(legs[1]);

                _head = _robot->skeleton()->getBodyNode(head_link())->createEndEffector(head_link());
            }
        };
    } // namespace model
} // namespace icub

#endif