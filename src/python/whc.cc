#include "whc.hpp"

PYBIND11_MODULE(pywhc, m)
{
    using namespace whc::python;
    // Load dartpy
    // py::module::import("dartpy");
    // Load magnum math
    // py::module::import("magnum.math");

    m.doc() = "pywhc: Python API of whc";

    py_dynamics(m);
    py_kinematics(m);
    py_control(m);
}