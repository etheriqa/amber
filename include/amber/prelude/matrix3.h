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

#include <limits>

#include <boost/operators.hpp>

#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T>
class Matrix3
: private boost::multipliable<Matrix3<T>>
{
public:
  Matrix3() noexcept;
  Matrix3(
    T e11, T e12, T e13,
    T e21, T e22, T e23,
    T e31, T e32, T e33
  ) noexcept;

  Matrix3<T>& operator*=(const Matrix3<T>& m) noexcept;

  const Vector3<T> operator()(const Vector3<T>& v) const noexcept;

  Matrix3<T> Inverse() const noexcept;

private:
  T e11_, e12_, e13_;
  T e21_, e22_, e23_;
  T e31_, e32_, e33_;

  Matrix3(T e) noexcept;

  const T Determinant() const noexcept;
};



template <typename T>
Matrix3<T>::Matrix3() noexcept
: Matrix3(std::numeric_limits<T>::quiet_NaN())
{}

template <typename T>
Matrix3<T>::Matrix3(T e) noexcept
: Matrix3(e, e, e, e, e, e, e, e, e)
{}

template <typename T>
Matrix3<T>::Matrix3(
  T e11, T e12, T e13,
  T e21, T e22, T e23,
  T e31, T e32, T e33
) noexcept
: e11_(e11), e12_(e12), e13_(e13)
, e21_(e21), e22_(e22), e23_(e23)
, e31_(e31), e32_(e32), e33_(e33)
{}

template <typename T>
Matrix3<T>&
Matrix3<T>::operator*=(const Matrix3<T>& m) noexcept
{
  return *this = Matrix3<T>(
    e11_ * m.e11_ + e12_ * m.e21_ + e13_ * m.e31_,
    e11_ * m.e12_ + e12_ * m.e22_ + e13_ * m.e32_,
    e11_ * m.e13_ + e12_ * m.e23_ + e13_ * m.e33_,
    e21_ * m.e11_ + e22_ * m.e21_ + e23_ * m.e31_,
    e21_ * m.e12_ + e22_ * m.e22_ + e23_ * m.e32_,
    e21_ * m.e13_ + e22_ * m.e23_ + e23_ * m.e33_,
    e31_ * m.e11_ + e32_ * m.e21_ + e33_ * m.e31_,
    e31_ * m.e12_ + e32_ * m.e22_ + e33_ * m.e32_,
    e31_ * m.e13_ + e32_ * m.e23_ + e33_ * m.e33_
  );
}

template <typename T>
const Vector3<T>
Matrix3<T>::operator()(const Vector3<T>& v) const noexcept
{
  return Vector3<T>(
    e11_ * v.X() + e12_ * v.Y() + e13_ * v.Z(),
    e21_ * v.X() + e22_ * v.Y() + e23_ * v.Z(),
    e31_ * v.X() + e32_ * v.Y() + e33_ * v.Z()
  );
}

template <typename T>
Matrix3<T>
Matrix3<T>::Inverse() const noexcept
{
  const auto d = Determinant();
  if (d == 0) {
    return Matrix3<T>();
  }

  const auto dinv = 1 / d;
  return Matrix3<T>(
    + dinv * (e22_ * e33_ - e23_ * e32_),
    - dinv * (e12_ * e33_ - e13_ * e32_),
    + dinv * (e12_ * e23_ - e13_ * e22_),
    - dinv * (e21_ * e33_ - e23_ * e31_),
    + dinv * (e11_ * e33_ - e13_ * e31_),
    - dinv * (e11_ * e23_ - e13_ * e21_),
    + dinv * (e21_ * e32_ - e22_ * e31_),
    - dinv * (e11_ * e32_ - e12_ * e31_),
    + dinv * (e11_ * e22_ - e12_ * e21_)
  );
}

template <typename T>
const T
Matrix3<T>::Determinant() const noexcept
{
  return
    + e11_ * (e22_ * e33_ - e23_ * e32_)
    - e21_ * (e12_ * e33_ - e13_ * e32_)
    + e31_ * (e12_ * e23_ - e13_ * e22_)
    ;
}

}
}
