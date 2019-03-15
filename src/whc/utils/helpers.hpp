#ifndef WHC_UTILS_HELPERS_HPP
#define WHC_UTILS_HELPERS_HPP

#include <memory>

namespace whc {
    namespace utils {
        template <typename T, typename... Args>
        std::unique_ptr<T> make_unique(Args... args)
        {
            return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
        }
    } // namespace utils
} // namespace whc

#endif