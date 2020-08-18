#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace whc {
    namespace python {
        void py_dynamics(py::module& m);
        void py_kinematics(py::module& m);
        void py_control(py::module& m);
    } // namespace python
} // namespace whc