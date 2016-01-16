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

#include <cmath>

#include <boost/operators.hpp>

namespace amber {
namespace prelude {

/** Two-dimentional vector.
 */
template <typename T>
class Vector2
: private boost::field_operators<Vector2<T>>
{
public:
  /** Constructors.
   */
  Vector2() noexcept;
  Vector2(const T& xy) noexcept;
  Vector2(const T& x, const T& y) noexcept;

  /** Operations.
   */
  const Vector2<T> operator+() const noexcept;
  const Vector2<T> operator-() const noexcept;
  Vector2<T>& operator+=(const Vector2<T>& v) noexcept;
  Vector2<T>& operator-=(const Vector2<T>& v) noexcept;
  Vector2<T>& operator*=(const Vector2<T>& v) noexcept;
  Vector2<T>& operator/=(const Vector2<T>& v) noexcept;

  /** Element queries.
   */
  const T X() const noexcept { return x_; }
  const T Y() const noexcept { return y_; }

private:
  T x_, y_;
};

/**
 * Sum of elements.
 */
template <typename T>
const T
Sum(const Vector2<T>& v) noexcept;

/** Minimum of elements.
 */
template <typename T>
const T
Min(const Vector2<T>& v) noexcept;

/** Maximum of elements.
 */
template <typename T>
const T
Max(const Vector2<T>& v) noexcept;

/** Whether it has at least one positive element or not.
 */
template <typename T>
bool
Positive(const Vector2<T>& v) noexcept;





template <typename T>
Vector2<T>::Vector2() noexcept
: x_(0)
, y_(0)
{}

template <typename T>
Vector2<T>::Vector2(const T& xy) noexcept
: x_(xy)
, y_(xy)
{}

template <typename T>
Vector2<T>::Vector2(const T& x, const T& y) noexcept
: x_(x)
, y_(y)
{}

template <typename T>
const Vector2<T>
Vector2<T>::operator+() const noexcept
{
  return *this;
}

template <typename T>
const Vector2<T>
Vector2<T>::operator-() const noexcept
{
  return Vector2<T>(-x_, -y_);
}

template <typename T>
Vector2<T>&
Vector2<T>::operator+=(const Vector2<T>& v) noexcept
{
  return this->operator=(Vector2<T>(x_ + v.x_, y_ + v.y_));
}

template <typename T>
Vector2<T>&
Vector2<T>::operator-=(const Vector2<T>& v) noexcept
{
  return this->operator=(Vector2<T>(x_ - v.x_, y_ - v.y_));
}

template <typename T>
Vector2<T>&
Vector2<T>::operator*=(const Vector2<T>& v) noexcept
{
  return this->operator=(Vector2<T>(x_ * v.x_, y_ * v.y_));
}

template <typename T>
Vector2<T>&
Vector2<T>::operator/=(const Vector2<T>& v) noexcept
{
  return this->operator=(Vector2<T>(x_ / v.x_, y_ / v.y_));
}

template <typename T>
const T
Sum(const Vector2<T>& v) noexcept
{
  return v.X() + v.Y();
}

template <typename T>
const T
Min(const Vector2<T>& v) noexcept
{
  return std::min(v.X(), v.Y());
}

template <typename T>
const T
Max(const Vector2<T>& v) noexcept
{
  return std::max(v.X(), v.Y());
}

template <typename T>
bool
Positive(const Vector2<T>& v) noexcept
{
  return Max(v) > static_cast<T>(0);
}

}
}
