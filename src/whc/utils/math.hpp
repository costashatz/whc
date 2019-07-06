#ifndef WHC_UTILS_MATH_HPP
#define WHC_UTILS_MATH_HPP

#include <Eigen/Core>

namespace whc {
    namespace utils {
        Eigen::Vector3d rotation_error(const Eigen::Matrix3d& R_desired, const Eigen::Matrix3d& R_current);
    } // namespace utils
} // namespace whc

#endif