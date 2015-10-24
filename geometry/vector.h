/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <ostream>
#include <tuple>

namespace amber {
namespace geometry {

template <typename T>
class Vector3 {
public:
  using value_type = T;

private:
  std::array<T, 3> values_;

public:
  Vector3() noexcept {}
  explicit Vector3(const T& value) noexcept { values_.fill(value); }
  Vector3(const T& x, const T& y, const T& z) noexcept : values_({{x, y, z}}) {}

  T& x() noexcept { return values_[0]; }
  const T& x() const noexcept { return values_[0]; }
  T& y() noexcept { return values_[1]; }
  const T& y() const noexcept { return values_[1]; }
  T& z() noexcept { return values_[2]; }
  const T& z() const noexcept { return values_[2]; }

  T& operator[](size_t pos) { return values_[pos]; }
  const T& operator[](size_t pos) const { return values_[pos]; }

  T min() const noexcept { return std::min({x(), y(), z()}); }
  T max() const noexcept { return std::max({x(), y(), z()}); }
  T sum() const noexcept { return x() + y() + z(); }
  T avg() const noexcept { return sum() / 3; }
  T squaredLength() const noexcept { return dot(*this, *this); }
  T length() const noexcept { return std::sqrt(squaredLength()); }

  template <typename U>
  Vector3<U> cast() const noexcept { return Vector3<U>(x(), y(), z()); }

  template <typename U>
  Vector3<T>& operator+=(const U& u) noexcept { return *this = *this + u; }

  template <typename U>
  Vector3<T>& operator-=(const U& u) noexcept { return *this = *this - u; }

  template <typename U>
  Vector3<T>& operator*=(const U& u) noexcept { return *this = *this * u; }

  template <typename U>
  Vector3<T>& operator/=(const U& u) noexcept { return *this = *this / u; }
};

template <typename T>
std::ostream& operator<<(std::ostream& os, const Vector3<T>& v) {
  os << "(" << v.x() << " " << v.y() << " " << v.z() << ")";
  return os;
}

template <typename T>
const Vector3<T>& operator+(const Vector3<T>& v) noexcept {
  return v;
}

template <typename T>
Vector3<T> operator-(const Vector3<T>& v) noexcept {
  return Vector3<T>(-v.x(), -v.y(), -v.z());
}

template <typename T>
Vector3<T> operator+(const Vector3<T>& u, const Vector3<T>& v) noexcept {
  return Vector3<T>(u.x() + v.x(), u.y() + v.y(), u.z() + v.z());
}

template <typename T, typename U>
Vector3<T> operator+(const Vector3<T>& v, const U& s) noexcept {
  return Vector3<T>(v.x() + s, v.y() + s, v.z() + s);
}

template <typename T, typename U>
Vector3<T> operator+(const U& s, const Vector3<T>& v) noexcept {
  return Vector3<T>(s + v.x(), s + v.y(), s + v.z());
}

template <typename T>
Vector3<T> operator-(const Vector3<T>& u, const Vector3<T>& v) noexcept {
  return Vector3<T>(u.x() - v.x(), u.y() - v.y(), u.z() - v.z());
}

template <typename T, typename U>
Vector3<T> operator-(const Vector3<T>& v, const U& s) noexcept {
  return Vector3<T>(v.x() - s, v.y() - s, v.z() - s);
}

template <typename T, typename U>
Vector3<T> operator-(const U& s, const Vector3<T>& v) noexcept {
  return Vector3<T>(s - v.x(), s - v.y(), s - v.z());
}

template <typename T>
Vector3<T> operator*(const Vector3<T>& u, const Vector3<T>& v) noexcept {
  return Vector3<T>(u.x() * v.x(), u.y() * v.y(), u.z() * v.z());
}

template <typename T, typename U>
Vector3<T> operator*(const Vector3<T>& v, const U& s) noexcept {
  return Vector3<T>(v.x() * s, v.y() * s, v.z() * s);
}

template <typename T, typename U>
Vector3<T> operator*(const U& s, const Vector3<T>& v) noexcept {
  return Vector3<T>(s * v.x(), s * v.y(), s * v.z());
}

template <typename T>
Vector3<T> operator/(const Vector3<T>& u, const Vector3<T>& v) noexcept {
  return Vector3<T>(u.x() / v.x(), u.y() / v.y(), u.z() / v.z());
}

template <typename T, typename U>
Vector3<T> operator/(const Vector3<T>& v, const U& s) noexcept {
  return Vector3<T>(v.x() / s, v.y() / s, v.z() / s);
}

template <typename T, typename U>
Vector3<T> operator/(const U& s, const Vector3<T>& v) noexcept {
  return Vector3<T>(s / v.x(), s / v.y(), s / v.z());
}

template <typename T>
T dot(const Vector3<T>& u, const Vector3<T>& v) noexcept {
  return u.x() * v.x() + u.y() * v.y() + u.z() * v.z();
}

template <typename T>
Vector3<T> normalize(const Vector3<T>& v) noexcept {
  return v / v.length();
}

template <typename T>
Vector3<T> cross(const Vector3<T>& u, const Vector3<T>& v) noexcept
{
  return Vector3<T>(
    u.y() * v.z() - u.z() * v.y(),
    u.z() * v.x() - u.x() * v.z(),
    u.x() * v.y() - u.y() * v.x()
  );
}

template <typename T>
std::tuple<Vector3<T>, Vector3<T>>
orthonormalBasis(const Vector3<T>& w) noexcept
{
  const auto u = normalize(cross(w,
                                 std::abs(w.x()) < std::abs(w.y())
                                 ? Vector3<T>(1, 0, 0)
                                 : Vector3<T>(0, 1, 0)));
  const auto v = normalize(cross(w, u));
  return std::make_tuple(u, v);
}

}
}
