#include "whc.hpp"

#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>

#include <whc/dynamics/constraint/constraints.hpp>
#include <whc/dynamics/solver/id_solver.hpp>
#include <whc/dynamics/task/tasks.hpp>

namespace robot_dart {
    namespace python {
        void py_dynamics(py::module& m)
        {
            using namespace whc;
            using namespace whc::dyn;

            auto sm = m.def_submodule("dyn");
            auto usm = m.def_submodule("utils");

            // Tasks
            auto tsm = sm.def_submodule("task");
            // AccelerationTask
            py::class_<task::AccelerationTask>(tsm, "AccelerationTask")
                .def(py::init<const dart::dynamics::SkeletonPtr&, const std::string&, const Eigen::VectorXd&, double>(),
                    py::arg("skeleton"),
                    py::arg("body_name"),
                    py::arg("desired"),
                    py::arg("weight") = 1.)

                .def(py::init<const dart::dynamics::SkeletonPtr&, const std::string&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
                    py::arg("skeleton"),
                    py::arg("body_name"),
                    py::arg("desired"),
                    py::arg("weights"))

                .def("set_desired", &task::AccelerationTask::set_desired)
                .def("get_desired", &task::AccelerationTask::get_desired)

                .def("set_weights", &task::AccelerationTask::set_weights)
                .def("get_weights", &task::AccelerationTask::get_weights);

            // COMAccelerationTask
            py::class_<task::COMAccelerationTask>(tsm, "COMAccelerationTask")
                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, double>(),
                    py::arg("skeleton"),
                    py::arg("desired"),
                    py::arg("weight") = 1.)

                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
                    py::arg("skeleton"),
                    py::arg("desired"),
                    py::arg("weights"))

                .def("set_desired", &task::COMAccelerationTask::set_desired)
                .def("get_desired", &task::COMAccelerationTask::get_desired)

                .def("set_weights", &task::COMAccelerationTask::set_weights)
                .def("get_weights", &task::COMAccelerationTask::get_weights);

            // DirectTrackingTask
            py::class_<task::DirectTrackingTask>(tsm, "DirectTrackingTask")
                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, double>(),
                    py::arg("skeleton"),
                    py::arg("desired"),
                    py::arg("weight") = 1.)

                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
                    py::arg("skeleton"),
                    py::arg("desired"),
                    py::arg("weights"))

                .def("set_desired", &task::DirectTrackingTask::set_desired)
                .def("get_desired", &task::DirectTrackingTask::get_desired)

                .def("set_weights", &task::DirectTrackingTask::set_weights)
                .def("get_weights", &task::DirectTrackingTask::get_weights);

            // TauDiffTask
            py::class_<task::TauDiffTask>(tsm, "TauDiffTask")
                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, double>(),
                    py::arg("skeleton"),
                    py::arg("prev_tau"),
                    py::arg("weight") = 1.)

                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
                    py::arg("skeleton"),
                    py::arg("prev_tau"),
                    py::arg("weights"))

                .def("set_desired", &task::TauDiffTask::set_desired)
                .def("get_desired", &task::TauDiffTask::get_desired)

                .def("set_weights", &task::TauDiffTask::set_weights)
                .def("get_weights", &task::TauDiffTask::get_weights);

            // PostureTask
            py::class_<task::PostureTask>(tsm, "PostureTask")
                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, double, bool>(),
                    py::arg("skeleton"),
                    py::arg("desired"),
                    py::arg("weight") = 1.,
                    py::arg("floating_base") = true)

                .def(py::init<const dart::dynamics::SkeletonPtr&, const Eigen::VectorXd&, const Eigen::VectorXd&, bool>(),
                    py::arg("skeleton"),
                    py::arg("desired"),
                    py::arg("weights"),
                    py::arg("floating_base") = true)

                .def("set_desired", &task::PostureTask::set_desired)
                .def("get_desired", &task::PostureTask::get_desired)

                .def("set_weights", &task::PostureTask::set_weights)
                .def("get_weights", &task::PostureTask::get_weights);

            // Constraints
            auto csm = sm.def_submodule("constraint");
            // DynamicsConstraint
            py::class_<constraint::DynamicsConstraint>(csm, "DynamicsConstraint")
                .def(py::init<const dart::dynamics::SkeletonPtr&, bool>(),
                    py::arg("skeleton"),
                    py::arg("floating_base") = true);

            // Contact
            py::class_<utils::Contact>(usm, "Contact")
                .def(py::init<const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&, double, double, bool, double, double, double, double>(),
                    py::arg("nz"),
                    py::arg("nx"),
                    py::arg("ny"),
                    py::arg("min"),
                    py::arg("max"),
                    py::arg("mu_s"),
                    py::arg("mu_k"),
                    py::arg("cop_constraint") = false,
                    py::arg("d_y_min"),
                    py::arg("d_y_max"),
                    py::arg("d_x_min"),
                    py::arg("d_x_max"));

            // ContactConstraint
            py::class_<constraint::ContactConstraint>(csm, "ContactConstraint")
                .def(py::init<const dart::dynamics::SkeletonPtr&, const std::string&, const utils::Contact&>(),
                    py::arg("skeleton"),
                    py::arg("body_name"),
                    py::arg("contact"));

            // JointLimitsConstraint
            py::class_<constraint::JointLimitsConstraint>(csm, "JointLimitsConstraint")
                .def(py::init<const dart::dynamics::SkeletonPtr&>(),
                    py::arg("skeleton"));

            // Solver
            auto ssm = sm.def_submodule("solver");

            // IDSolver
            py::class_<solver::IDSolver>(ssm, "IDSolver")
                .def(py::init<const dart::dynamics::SkeletonPtr&>())

                .def("solve", &solver::IDSolver::solve)

                .def("set_skeleton", &solver::IDSolver::set_skeleton)

                .def("clear_all", &solver::IDSolver::clear_all)

                // .def("add_task", &solver::IDSolver::add_task)
                // .def("add_constraint", &solver::IDSolver::add_constraint)

                .def("dim", &solver::IDSolver::dim)

                // .def("tasks", &solver::IDSolver::tasks, py::return_value_policy::reference)
                // .def("contacts", &solver::IDSolver::contacts, py::return_value_policy::reference)
                // .def("constraints", &solver::IDSolver::constraints, py::return_value_policy::reference)

                .def("solution", &solver::IDSolver::solution);
        }
    } // namespace python
} // namespace robot_dart
