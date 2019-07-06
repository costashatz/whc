#include <whc/utils/math.hpp>

#include <cmath>
#include <limits>

#include <dart/math/Geometry.hpp>

namespace whc {
    namespace utils {
        Eigen::Vector3d rotation_error(const Eigen::Matrix3d& R_desired, const Eigen::Matrix3d& R_current)
        {
            return dart::math::logMap(R_desired * R_current.transpose());
        }
    } // namespace utils
} // namespace whc