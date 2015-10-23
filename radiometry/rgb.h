#pragma once

#include <algorithm>
#include <array>

namespace amber {
namespace radiometry {

template <typename T>
class RGB {
public:
  using value_type = T;

private:
  std::array<T, 3> values_;

public:
  RGB() noexcept : RGB(T()) {}
  explicit RGB(const T& value) noexcept { values_.fill(value); }
  RGB(const T& r, const T& g, const T& b) noexcept : values_({{r, g, b}}) {}

  T& r() noexcept { return values_[0]; }
  const T& r() const noexcept { return values_[0]; }
  T& g() noexcept { return values_[1]; }
  const T& g() const noexcept { return values_[1]; }
  T& b() noexcept { return values_[2]; }
  const T& b() const noexcept { return values_[2]; }

  T min() const noexcept { return std::min({r(), g(), b()}); }
  T max() const noexcept { return std::max({r(), g(), b()}); }
  T sum() const noexcept { return r() + g() + b(); }
  T avg() const noexcept { return sum() / 3; }

  template <typename U>
  RGB<T>& operator+=(const U& u) noexcept {
    return *this = *this + u;
  }

  template <typename U>
  RGB<T>& operator-=(const U& u) noexcept {
    return *this = *this - u;
  }

  template <typename U>
  RGB<T>& operator*=(const U& u) noexcept {
    return *this = *this * u;
  }

  template <typename U>
  RGB<T>& operator/=(const U& u) noexcept {
    return *this = *this / u;
  }
};

template <typename T>
RGB<T> operator+(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() + y.r(), x.g() + y.g(), x.b() + y.b());
}

template <typename T>
RGB<T> operator*(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() * y.r(), x.g() * y.g(), x.b() * y.b());
}

template <typename T, typename U>
RGB<T> operator*(const RGB<T>& x, const U& k) noexcept {
  return RGB<T>(x.r() * k, x.g() * k, x.b() * k);
}

template <typename T>
RGB<T> operator/(const RGB<T>& x, const RGB<T>& y) noexcept {
  return RGB<T>(x.r() / y.r(), x.g() / y.g(), x.b() / y.b());
}

template <typename T, typename U>
RGB<T> operator/(const RGB<T>& x, const U& k) noexcept {
  return RGB<T>(x.r() / k, x.g() / k, x.b() / k);
}

}
}
