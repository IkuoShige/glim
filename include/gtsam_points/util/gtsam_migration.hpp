#pragma once

#include <memory>

namespace gtsam_points {

template <typename T>
using shared_ptr = std::shared_ptr<T>;

template <typename T>
using weak_ptr = std::weak_ptr<T>;

}  // namespace gtsam_points
