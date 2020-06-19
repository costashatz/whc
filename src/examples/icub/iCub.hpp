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
#ifndef WHC_EXAMPLES_ICUB_ICUB_HPP
#define WHC_EXAMPLES_ICUB_ICUB_HPP

#include <robot_dart/robot.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/DegreeOfFreedom.hpp>
#include <dart/dynamics/Joint.hpp>
#include <dart/dynamics/MeshShape.hpp>

namespace whc {
    namespace icub_example {
        class iCub {
        public:
            iCub(const std::string& name, const std::string& model = "iCubGazeboV2_5_plus")
            {
                _robot = std::make_shared<robot_dart::Robot>(std::string(RESPATH) + "/robots/" + model + "/model.urdf", _packages(), name);
                _fix_masses();
                if (model == "iCubNancy01")
                    _fix_visuals();
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

        protected:
            std::shared_ptr<robot_dart::Robot> _robot;

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
                    else if (bd->getMass() == 0.) {
                        bd->setMass(1e-8);
                        bd->setMomentOfInertia(1., 1., 1., 0., 0., 0.);
                    }
                }
            }

            void _fix_visuals()
            {
                for (size_t i = 0; i < _robot->skeleton()->getNumBodyNodes(); i++) {
                    auto bd = _robot->skeleton()->getBodyNode(i);
                    auto shapes = bd->getShapeNodesWith<dart::dynamics::VisualAspect>();
                    for (size_t j = 0; j < shapes.size(); j++) {
                        auto node = shapes[j];
                        auto shape = node->getShape();
                        if (shape->getType() == "MeshShape") {
                            auto mesh_shape = std::static_pointer_cast<dart::dynamics::MeshShape>(shape);
                            mesh_shape->setColorMode(dart::dynamics::MeshShape::ColorMode::MATERIAL_COLOR);
                        }
                    }
                }
            }

            std::vector<std::pair<std::string, std::string>> _packages()
            {
                return {std::make_pair("iCub", std::string(RESPATH))};
            }
        };
    } // namespace icub_example
} // namespace whc

#endif