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

#include <boost/operators.hpp>

#include "core/writer.h"

namespace amber {
namespace core {

template <typename T>
class Vector3
: public Writer,
  private boost::field_operators<Vector3<T>>
{
public:
  using value_type = T;

private:
  std::array<T, 3> values_;

public:
  Vector3() noexcept {}
  Vector3(T const& value) noexcept { values_.fill(value); }
  Vector3(T const& x, T const& y, T const& z) noexcept : values_({{x, y, z}}) {}

  template <typename U>
  Vector3(Vector3<U> const& v) noexcept : Vector3(v.x(), v.y(), v.z()) {}

  T const& x() const noexcept { return values_[0]; }
  T& x() noexcept { return values_[0]; }
  T const& y() const noexcept { return values_[1]; }
  T& y() noexcept { return values_[1]; }
  T const& z() const noexcept { return values_[2]; }
  T& z() noexcept { return values_[2]; }

  void Write(std::ostream& os) const noexcept {
    os << "(" << x() << " " << y() << " " << z() << ")";
  }

  T& operator[](size_t pos) { return values_[pos]; }
  T const& operator[](size_t pos) const { return values_[pos]; }

  Vector3<T> const& operator+() const noexcept
  {
    return *this;
  }

  Vector3<T> operator-() const noexcept
  {
    return Vector3<T>(-x(), -y(), -z());
  }

  Vector3<T>& operator+=(Vector3<T> const& v) noexcept
  {
    x() += v.x();
    y() += v.y();
    z() += v.z();
    return *this;
  }

  Vector3<T>& operator-=(Vector3<T> const& v) noexcept
  {
    x() -= v.x();
    y() -= v.y();
    z() -= v.z();
    return *this;
  }

  Vector3<T>& operator*=(Vector3<T> const& v) noexcept
  {
    x() *= v.x();
    y() *= v.y();
    z() *= v.z();
    return *this;
  }

  Vector3<T>& operator/=(Vector3<T> const& v) noexcept
  {
    x() /= v.x();
    y() /= v.y();
    z() /= v.z();
    return *this;
  }
};

template <typename T>
T
Dot(Vector3<T> const& u, Vector3<T> const& v) noexcept
{
  return u.x() * v.x() + u.y() * v.y() + u.z() * v.z();
}

template <typename T>
T
SquaredLength(Vector3<T> const& v) noexcept
{
  return Dot(v, v);
}

template <typename T>
T
Length(Vector3<T> const& v) noexcept
{
  return std::sqrt(SquaredLength(v));
}

template <typename T>
Vector3<T>
Normalize(Vector3<T> const& v) noexcept {
  return v / Length(v);
}

template <typename T>
Vector3<T>
Cross(Vector3<T> const& u, Vector3<T> const& v) noexcept
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
  auto const u = Normalize(Cross(
    w,
    std::abs(w.x()) < std::abs(w.y())
      ? Vector3<T>(1, 0, 0)
      : Vector3<T>(0, 1, 0)
  ));
  auto const v = Normalize(Cross(w, u));
  return std::make_tuple(u, v);
}

}
}
