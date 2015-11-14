// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

#include "core/writer.h"

namespace amber {
namespace core {

template <typename T>
class Vector3 : public Writer {
public:
  using value_type = T;

private:
  std::array<T, 3> values_;

public:
  Vector3() noexcept {}
  explicit Vector3(T const& value) noexcept { values_.fill(value); }
  Vector3(T const& x, T const& y, T const& z) noexcept : values_({{x, y, z}}) {}

  T& x() noexcept { return values_[0]; }
  T const& x() const noexcept { return values_[0]; }
  T& y() noexcept { return values_[1]; }
  T const& y() const noexcept { return values_[1]; }
  T& z() noexcept { return values_[2]; }
  T const& z() const noexcept { return values_[2]; }

  T& operator[](size_t pos) { return values_[pos]; }
  T const& operator[](size_t pos) const { return values_[pos]; }

  void Write(std::ostream& os) const noexcept {
    os << "(" << x() << " " << y() << " " << z() << ")";
  }

  T SquaredLength() const noexcept { return Dot(*this, *this); }
  T Length() const noexcept { return std::sqrt(SquaredLength()); }

  template <typename U>
  Vector3<U> cast() const noexcept { return Vector3<U>(x(), y(), z()); }

  template <typename U>
  Vector3<T>& operator+=(U const& u) noexcept { return *this = *this + u; }

  template <typename U>
  Vector3<T>& operator-=(U const& u) noexcept { return *this = *this - u; }

  template <typename U>
  Vector3<T>& operator*=(U const& u) noexcept { return *this = *this * u; }

  template <typename U>
  Vector3<T>& operator/=(U const& u) noexcept { return *this = *this / u; }
};

template <typename T>
Vector3<T> const& operator+(Vector3<T> const& v) noexcept {
  return v;
}

template <typename T>
Vector3<T> operator-(Vector3<T> const& v) noexcept {
  return Vector3<T>(-v.x(), -v.y(), -v.z());
}

template <typename T>
Vector3<T> operator+(Vector3<T> const& u, Vector3<T> const& v) noexcept {
  return Vector3<T>(u.x() + v.x(), u.y() + v.y(), u.z() + v.z());
}

template <typename T, typename U>
Vector3<T> operator+(Vector3<T> const& v, U const& s) noexcept {
  return Vector3<T>(v.x() + s, v.y() + s, v.z() + s);
}

template <typename T, typename U>
Vector3<T> operator+(U const& s, Vector3<T> const& v) noexcept {
  return Vector3<T>(s + v.x(), s + v.y(), s + v.z());
}

template <typename T>
Vector3<T> operator-(Vector3<T> const& u, Vector3<T> const& v) noexcept {
  return Vector3<T>(u.x() - v.x(), u.y() - v.y(), u.z() - v.z());
}

template <typename T, typename U>
Vector3<T> operator-(Vector3<T> const& v, U const& s) noexcept {
  return Vector3<T>(v.x() - s, v.y() - s, v.z() - s);
}

template <typename T, typename U>
Vector3<T> operator-(U const& s, Vector3<T> const& v) noexcept {
  return Vector3<T>(s - v.x(), s - v.y(), s - v.z());
}

template <typename T>
Vector3<T> operator*(Vector3<T> const& u, Vector3<T> const& v) noexcept {
  return Vector3<T>(u.x() * v.x(), u.y() * v.y(), u.z() * v.z());
}

template <typename T, typename U>
Vector3<T> operator*(Vector3<T> const& v, U const& s) noexcept {
  return Vector3<T>(v.x() * s, v.y() * s, v.z() * s);
}

template <typename T, typename U>
Vector3<T> operator*(U const& s, Vector3<T> const& v) noexcept {
  return Vector3<T>(s * v.x(), s * v.y(), s * v.z());
}

template <typename T>
Vector3<T> operator/(Vector3<T> const& u, Vector3<T> const& v) noexcept {
  return Vector3<T>(u.x() / v.x(), u.y() / v.y(), u.z() / v.z());
}

template <typename T, typename U>
Vector3<T> operator/(Vector3<T> const& v, U const& s) noexcept {
  return Vector3<T>(v.x() / s, v.y() / s, v.z() / s);
}

template <typename T, typename U>
Vector3<T> operator/(U const& s, Vector3<T> const& v) noexcept {
  return Vector3<T>(s / v.x(), s / v.y(), s / v.z());
}

template <typename T>
T Dot(Vector3<T> const& u, Vector3<T> const& v) noexcept {
  return u.x() * v.x() + u.y() * v.y() + u.z() * v.z();
}

template <typename T>
Vector3<T> Normalize(Vector3<T> const& v) noexcept {
  return v / v.Length();
}

template <typename T>
Vector3<T> Cross(Vector3<T> const& u, Vector3<T> const& v) noexcept
{
  return Vector3<T>(
    u.y() * v.z() - u.z() * v.y(),
    u.z() * v.x() - u.x() * v.z(),
    u.x() * v.y() - u.y() * v.x()
  );
}

template <typename T>
std::tuple<Vector3<T>, Vector3<T>>
OrthonormalBasis(Vector3<T> const& w) noexcept
{
  auto const u = Normalize(Cross(w,
                                 std::abs(w.x()) < std::abs(w.y())
                                 ? Vector3<T>(1, 0, 0)
                                 : Vector3<T>(0, 1, 0)));
  auto const v = Normalize(Cross(w, u));
  return std::make_tuple(u, v);
}

}
}
