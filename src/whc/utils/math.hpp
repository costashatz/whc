#ifndef WHC_UTILS_MATH_HPP
#define WHC_UTILS_MATH_HPP

#include <Eigen/Core>

namespace whc {
    namespace utils {
        // This is copied from: https://github.com/jrl-umi3218/SpaceVecAlg
        double sinc_inv(double x);
        Eigen::Vector3d rotation_error(const Eigen::Matrix3d& R_ab, const Eigen::Matrix3d& R_ac);
    } // namespace utils
} // namespace whc

#endif