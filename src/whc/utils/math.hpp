#ifndef WHC_UTILS_MATH_HPP
#define WHC_UTILS_MATH_HPP

#include <cmath>
#include <limits>

namespace whc {
    namespace utils {
        // This is copied from: https://github.com/jrl-umi3218/SpaceVecAlg
        double sinc_inv(double x)
        {
            const double taylor_0_bound = std::numeric_limits<double>::epsilon();
            const double taylor_2_bound = std::sqrt(taylor_0_bound);
            const double taylor_n_bound = std::sqrt(taylor_2_bound);

            // We use the 4th order taylor series around 0 of x/sin(x) to compute
            // this function:
            //      2      4
            //     x    7⋅x     ⎛ 6⎞
            // 1 + ── + ──── + O⎝x ⎠
            //     6    360
            // this approximation is valid around 0.
            // if x is far from 0, our approximation is not valid
            // since x^6 becomes non negligable we use the normal computation of the function
            // (i.e. taylor_2_bound^6 + taylor_0_bound == taylor_0_bound but
            //       taylor_n_bound^6 + taylor_0_bound != taylor_0).

            if (std::abs(x) >= taylor_n_bound)
                return (x / std::sin(x));

            // x is bellow taylor_n_bound so we don't care of the 6th order term of
            // the taylor series.
            // We set the 0 order term.
            double result = 1.;

            if (std::abs(x) >= taylor_0_bound) {
                // x is above the machine epsilon so x^2 is meaningful.
                double x2 = x * x;
                result += x2 / 6.;

                if (std::abs(x) >= taylor_2_bound) {
                    // x is upper the machine sqrt(epsilon) so x^4 is meaningful.
                    result += 7. * (x2 * x2) / 360.;
                }
            }

            return result;
        }

        Eigen::Vector3d rotation_error(const Eigen::Matrix3d& R_ab, const Eigen::Matrix3d& R_ac)
        {
            Eigen::Matrix3d E_bc = R_ac * R_ab.transpose();

            Eigen::Vector3d w;
            double acos_v = (E_bc.trace() - 1) * 0.5;
            double theta = std::acos(std::min(std::max(acos_v, -1.), 1.));

            w << -E_bc(2, 1) + E_bc(1, 2),
                -E_bc(0, 2) + E_bc(2, 0),
                -E_bc(1, 0) + E_bc(0, 1);

            w *= sinc_inv(theta) * 0.5;

            return R_ab.transpose() * w;
        }
    } // namespace utils
} // namespace whc

#endif