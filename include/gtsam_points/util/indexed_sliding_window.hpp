#pragma once

#include <algorithm>
#include <vector>

namespace gtsam_points {

template <typename T>
class IndexedSlidingWindow {
public:
  using Container = std::vector<T>;
  using iterator = typename Container::iterator;
  using const_iterator = typename Container::const_iterator;

  bool empty() const { return values.empty(); }
  size_t size() const { return values.size(); }
  void clear() { values.clear(); }
  void push_back(const T& value) { values.push_back(value); }
  T& front() { return values.front(); }
  const T& front() const { return values.front(); }
  T& back() { return values.back(); }
  const T& back() const { return values.back(); }

  T& operator[](size_t i) { return values[i]; }
  const T& operator[](size_t i) const { return values[i]; }

  iterator begin() { return values.begin(); }
  iterator end() { return values.end(); }
  const_iterator begin() const { return values.begin(); }
  const_iterator end() const { return values.end(); }

  const_iterator inner_begin() const { return std::find_if(values.begin(), values.end(), [](const T& value) { return static_cast<bool>(value); }); }
  const_iterator inner_end() const { return values.end(); }

  size_t inner_size() const {
    return std::count_if(values.begin(), values.end(), [](const T& value) { return static_cast<bool>(value); });
  }

private:
  Container values;
};

}  // namespace gtsam_points
