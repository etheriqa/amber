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

/** One-dimentional vector.
 */
template <typename T>
class Vector1
: private boost::field_operators<Vector1<T>>
{
public:
  /** Constructors.
   */
  Vector1() noexcept;
  Vector1(const T& x) noexcept;

  /** Operations.
   */
  const Vector1<T> operator+() const noexcept;
  const Vector1<T> operator-() const noexcept;
  Vector1<T>& operator+=(const Vector1<T>& v) noexcept;
  Vector1<T>& operator-=(const Vector1<T>& v) noexcept;
  Vector1<T>& operator*=(const Vector1<T>& v) noexcept;
  Vector1<T>& operator/=(const Vector1<T>& v) noexcept;

  /** Element query.
   */
  const T X() const noexcept { return x_; }

private:
  T x_;
};

/**
 * Sum of elements.
 */
template <typename T>
const T
Sum(const Vector1<T>& v) noexcept;

/** Minimum of elements.
 */
template <typename T>
const T
Min(const Vector1<T>& v) noexcept;

/** Maximum of elements.
 */
template <typename T>
const T
Max(const Vector1<T>& v) noexcept;

/** Whether it has at least one positive element or not.
 */
template <typename T>
bool
Positive(const Vector1<T>& v) noexcept;





template <typename T>
Vector1<T>::Vector1() noexcept
: Vector1(0)
{}

template <typename T>
Vector1<T>::Vector1(const T& x) noexcept
: x_(0)
{}

template <typename T>
const Vector1<T>
Vector1<T>::operator+() const noexcept
{
  return *this;
}

template <typename T>
const Vector1<T>
Vector1<T>::operator-() const noexcept
{
  return Vector1<T>(-x_);
}

template <typename T>
Vector1<T>&
Vector1<T>::operator+=(const Vector1<T>& v) noexcept
{
  return this->operator=(Vector1<T>(x_ + v.x_));
}

template <typename T>
Vector1<T>&
Vector1<T>::operator-=(const Vector1<T>& v) noexcept
{
  return this->operator=(Vector1<T>(x_ - v.x_));
}

template <typename T>
Vector1<T>&
Vector1<T>::operator*=(const Vector1<T>& v) noexcept
{
  return this->operator=(Vector1<T>(x_ * v.x_));
}

template <typename T>
Vector1<T>&
Vector1<T>::operator/=(const Vector1<T>& v) noexcept
{
  return this->operator=(Vector1<T>(x_ / v.x_));
}

template <typename T>
const T
Sum(const Vector1<T>& v) noexcept
{
  return v.X();
}

template <typename T>
const T
Min(const Vector1<T>& v) noexcept
{
  return v.X();
}

template <typename T>
const T
Max(const Vector1<T>& v) noexcept
{
  return v.X();
}

template <typename T>
bool
Positive(const Vector1<T>& v) noexcept
{
  return Max(v) > static_cast<T>(0);
}

}
}
