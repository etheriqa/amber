// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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
#include <cmath>
#include <tuple>

#include <boost/operators.hpp>

#include "amber/prelude/vector2.h"

namespace amber {
namespace prelude {

/** Three-dimentional vector.
 */
template <typename T>
class Vector3
: private boost::field_operators<Vector3<T>>
{
public:
  /** Constructors.
   */
  Vector3() noexcept;
  Vector3(const T& xyz) noexcept;
  Vector3(const T& x, const T& y, const T& z) noexcept;
  Vector3(const Vector2<T>& xy, const T& z) noexcept;

  /** Operations.
   */
  const Vector3<T> operator+() const noexcept;
  const Vector3<T> operator-() const noexcept;
  Vector3<T>& operator+=(const Vector3<T>& v) noexcept;
  Vector3<T>& operator-=(const Vector3<T>& v) noexcept;
  Vector3<T>& operator*=(const Vector3<T>& v) noexcept;
  Vector3<T>& operator/=(const Vector3<T>& v) noexcept;

  /** Element queries.
   */
  const T X() const noexcept { return x_; }
  const T Y() const noexcept { return y_; }
  const T Z() const noexcept { return z_; }

private:
  T x_, y_, z_;
};

/** Three-dimentional unit (normalized) vector.
 */
template <typename T>
class UnitVector3
: public Vector3<T>
{
public:
  /** Constructors.
   */
  UnitVector3() noexcept;
  UnitVector3(const T& x, const T& y, const T& z) noexcept;
  explicit UnitVector3(const Vector3<T>& v) noexcept;

  /** Operations.
   */
  const UnitVector3<T> operator+() const noexcept;
  const UnitVector3<T> operator-() const noexcept;
};

/** Drops Z element.
 */
template <typename T>
const Vector2<T>
XY(const Vector3<T>& v) noexcept;

/** Sum of elements.
 */
template <typename T>
const T
Sum(const Vector3<T>& v) noexcept;

/** Minimum of elements.
 */
template <typename T>
const T
Min(const Vector3<T>& v) noexcept;

/** Maximum of elements.
 */
template <typename T>
const T
Max(const Vector3<T>& v) noexcept;

/** Whether it has at least one positive element or not.
 */
template <typename T>
bool
Positive(const Vector3<T>& v) noexcept;

/** Dot product.
 */
template <typename T>
const T
Dot(const Vector3<T>& u, const Vector3<T>& v) noexcept;

/** Squared length.
 */
template <typename T>
const T
SquaredLength(const Vector3<T>& v) noexcept;

/** Length.
 */
template <typename T>
const T
Length(const Vector3<T>& v) noexcept;

/** Normalizes a vector.
 */
template <typename T>
const UnitVector3<T>
Normalize(const Vector3<T>& v) noexcept;

/** Cross product.
 */
template <typename T>
const Vector3<T>
Cross(const Vector3<T>& u, const Vector3<T>& v) noexcept;

/** Finds two orthonormal basis vectors.
 */
template <typename T>
std::tuple<UnitVector3<T>, UnitVector3<T>>
OrthonormalBasis(const UnitVector3<T>& w) noexcept;





template <typename T>
Vector3<T>::Vector3() noexcept
: x_(0)
, y_(0)
, z_(0)
{}

template <typename T>
Vector3<T>::Vector3(const T& xyz) noexcept
: x_(xyz)
, y_(xyz)
, z_(xyz)
{}

template <typename T>
Vector3<T>::Vector3(const T& x, const T& y, const T& z) noexcept
: x_(x)
, y_(y)
, z_(z)
{}

template <typename T>
Vector3<T>::Vector3(const Vector2<T>& xy, const T& z) noexcept
: x_(xy.X())
, y_(xy.Y())
, z_(z)
{}

template <typename T>
const Vector3<T>
Vector3<T>::operator+() const noexcept
{
  return *this;
}

template <typename T>
const Vector3<T>
Vector3<T>::operator-() const noexcept
{
  return Vector3<T>(-x_, -y_, -z_);
}

template <typename T>
Vector3<T>&
Vector3<T>::operator+=(const Vector3<T>& v) noexcept
{
  return this->operator=(Vector3<T>(x_ + v.x_, y_ + v.y_, z_ + v.z_));
}

template <typename T>
Vector3<T>&
Vector3<T>::operator-=(const Vector3<T>& v) noexcept
{
  return this->operator=(Vector3<T>(x_ - v.x_, y_ - v.y_, z_ - v.z_));
}

template <typename T>
Vector3<T>&
Vector3<T>::operator*=(const Vector3<T>& v) noexcept
{
  return this->operator=(Vector3<T>(x_ * v.x_, y_ * v.y_, z_ * v.z_));
}

template <typename T>
Vector3<T>&
Vector3<T>::operator/=(const Vector3<T>& v) noexcept
{
  return this->operator=(Vector3<T>(x_ / v.x_, y_ / v.y_, z_ / v.z_));
}

template <typename T>
UnitVector3<T>::UnitVector3() noexcept
: Vector3<T>()
{}

template <typename T>
UnitVector3<T>::UnitVector3(const T& x, const T& y, const T& z) noexcept
: Vector3<T>(x, y, z)
{}

template <typename T>
UnitVector3<T>::UnitVector3(const Vector3<T>& v) noexcept
: Vector3<T>(v.X(), v.Y(), v.Z())
{}

template <typename T>
const UnitVector3<T>
UnitVector3<T>::operator+() const noexcept
{
  return *this;
}

template <typename T>
const UnitVector3<T>
UnitVector3<T>::operator-() const noexcept
{
  return UnitVector3<T>(-this->X(), -this->Y(), -this->Z());
}

template <typename T>
const Vector2<T>
XY(const Vector3<T>& v) noexcept
{
  return Vector2<T>(v.X(), v.Y());
}

template <typename T>
const T
Sum(const Vector3<T>& v) noexcept
{
  return v.X() + v.Y() + v.Z();
}

template <typename T>
const T
Min(const Vector3<T>& v) noexcept
{
  return std::min({v.X(), v.Y(), v.Z()});
}

template <typename T>
const T
Max(const Vector3<T>& v) noexcept
{
  return std::max({v.X(), v.Y(), v.Z()});
}

template <typename T>
bool
Positive(const Vector3<T>& v) noexcept
{
  return Max(v) > static_cast<T>(0);
}

template <typename T>
const T
Dot(const Vector3<T>& u, const Vector3<T>& v) noexcept
{
  return u.X() * v.X() + u.Y() * v.Y() + u.Z() * v.Z();
}

template <typename T>
const T
SquaredLength(const Vector3<T>& v) noexcept
{
  return Dot(v, v);
}

template <typename T>
const T
Length(const Vector3<T>& v) noexcept
{
  return std::sqrt(SquaredLength(v));
}

template <typename T>
const UnitVector3<T>
Normalize(const Vector3<T>& v) noexcept
{
  const auto length = Length(v);
  return UnitVector3<T>(v.X() / length, v.Y() / length, v.Z() / length);
}

template <typename T>
const Vector3<T>
Cross(const Vector3<T>& u, const Vector3<T>& v) noexcept
{
  return Vector3<T>(
    u.Y() * v.Z() - u.Z() * v.Y(),
    u.Z() * v.X() - u.X() * v.Z(),
    u.X() * v.Y() - u.Y() * v.X()
  );
}

template <typename T>
std::tuple<UnitVector3<T>, UnitVector3<T>>
OrthonormalBasis(const UnitVector3<T>& w) noexcept
{
  const auto u = Normalize(Cross(
    w,
    std::abs(w.X()) < std::abs(w.Y())
      ? Vector3<T>(1, 0, 0)
      : Vector3<T>(0, 1, 0)
  ));
  const auto v = Normalize(Cross(w, u));
  return std::make_tuple(u, v);
}

}
}
