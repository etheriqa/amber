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

#include "amber/prelude/matrix3.h"
#include "amber/prelude/ray.h"
#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T>
class Matrix4
{
public:
  Matrix4() noexcept;
  Matrix4(
    T e11, T e12, T e13, T e14,
    T e21, T e22, T e23, T e24,
    T e31, T e32, T e33, T e34,
    T e41, T e42, T e43, T e44
  ) noexcept;

  operator Matrix3<T>() const noexcept;

  const Vector3<T> operator()(const Vector3<T>& v) const noexcept;
  Ray<T> operator()(const Ray<T>& ray) const noexcept;

  Matrix4<T> Inverse() const noexcept;

private:
  T e11_, e12_, e13_, e14_;
  T e21_, e22_, e23_, e24_;
  T e31_, e32_, e33_, e34_;
  T e41_, e42_, e43_, e44_;

  Matrix4(T e) noexcept;

  const T Determinant() const noexcept;
};



template <typename T>
Matrix4<T>::Matrix4() noexcept
: Matrix4(std::numeric_limits<T>::quiet_NaN())
{}

template <typename T>
Matrix4<T>::Matrix4(T e) noexcept
: Matrix4(e, e, e, e, e, e, e, e, e, e, e, e, e, e, e, e)
{}

template <typename T>
Matrix4<T>::Matrix4(
  T e11, T e12, T e13, T e14,
  T e21, T e22, T e23, T e24,
  T e31, T e32, T e33, T e34,
  T e41, T e42, T e43, T e44
) noexcept
: e11_(e11), e12_(e12), e13_(e13), e14_(e14)
, e21_(e21), e22_(e22), e23_(e23), e24_(e24)
, e31_(e31), e32_(e32), e33_(e33), e34_(e34)
, e41_(e41), e42_(e42), e43_(e43), e44_(e44)
{}

template <typename T>
Matrix4<T>::operator Matrix3<T>() const noexcept
{
  return Matrix3<T>(
    e11_, e12_, e13_,
    e21_, e22_, e23_,
    e31_, e32_, e33_
  );
}

template <typename T>
const Vector3<T>
Matrix4<T>::operator()(const Vector3<T>& v) const noexcept
{
  return Vector3<T>(
    e11_ * v.X() + e12_ * v.Y() + e13_ * v.Z() + e14_,
    e21_ * v.X() + e22_ * v.Y() + e23_ * v.Z() + e24_,
    e31_ * v.X() + e32_ * v.Y() + e33_ * v.Z() + e34_
  );
}

template <typename T>
Ray<T>
Matrix4<T>::operator()(const Ray<T>& ray) const noexcept
{
  return Ray<T>(
    (*this)(ray.Origin()),
    static_cast<Matrix3<T>>(*this)(ray.Direction())
  );
}

template <typename T>
Matrix4<T>
Matrix4<T>::Inverse() const noexcept
{
  const auto d = Determinant();
  if (d == 0) {
    return Matrix4<T>();
  }

  const auto dinv = 1 / d;
  return Matrix4<T>(
    +dinv*(+e22_*(e33_*e44_-e34_*e43_)-e23_*(e32_*e44_-e34_*e42_)+e24_*(e32_*e43_-e33_*e42_)),
    -dinv*(+e12_*(e33_*e44_-e34_*e43_)-e13_*(e32_*e44_-e34_*e42_)+e14_*(e32_*e43_-e33_*e42_)),
    +dinv*(+e12_*(e23_*e44_-e24_*e43_)-e13_*(e22_*e44_-e24_*e42_)+e14_*(e22_*e43_-e23_*e42_)),
    -dinv*(+e12_*(e23_*e34_-e24_*e33_)-e13_*(e22_*e34_-e24_*e32_)+e14_*(e22_*e33_-e23_*e32_)),
    -dinv*(+e21_*(e33_*e44_-e34_*e43_)-e23_*(e31_*e44_-e34_*e41_)+e24_*(e31_*e43_-e33_*e41_)),
    +dinv*(+e11_*(e33_*e44_-e34_*e43_)-e13_*(e31_*e44_-e34_*e41_)+e14_*(e31_*e43_-e33_*e41_)),
    -dinv*(+e11_*(e23_*e44_-e24_*e43_)-e13_*(e21_*e44_-e24_*e41_)+e14_*(e21_*e43_-e23_*e41_)),
    +dinv*(+e11_*(e23_*e34_-e24_*e33_)-e13_*(e21_*e34_-e24_*e31_)+e14_*(e21_*e33_-e23_*e31_)),
    +dinv*(+e21_*(e32_*e44_-e34_*e42_)-e22_*(e31_*e44_-e34_*e41_)+e24_*(e31_*e42_-e32_*e41_)),
    -dinv*(+e11_*(e32_*e44_-e34_*e42_)-e12_*(e31_*e44_-e34_*e41_)+e14_*(e31_*e42_-e32_*e41_)),
    +dinv*(+e11_*(e22_*e44_-e24_*e42_)-e12_*(e21_*e44_-e24_*e41_)+e14_*(e21_*e42_-e22_*e41_)),
    -dinv*(+e11_*(e22_*e34_-e24_*e32_)-e12_*(e21_*e34_-e24_*e31_)+e14_*(e21_*e32_-e22_*e31_)),
    -dinv*(+e21_*(e32_*e43_-e33_*e42_)-e22_*(e31_*e43_-e33_*e41_)+e23_*(e31_*e42_-e32_*e41_)),
    +dinv*(+e11_*(e32_*e43_-e33_*e42_)-e12_*(e31_*e43_-e33_*e41_)+e13_*(e31_*e42_-e32_*e41_)),
    -dinv*(+e11_*(e22_*e43_-e23_*e42_)-e12_*(e21_*e43_-e23_*e41_)+e13_*(e21_*e42_-e22_*e41_)),
    +dinv*(+e11_*(e22_*e33_-e23_*e32_)-e12_*(e21_*e33_-e23_*e31_)+e13_*(e21_*e32_-e22_*e31_))
  );
}

template <typename T>
const T
Matrix4<T>::Determinant() const noexcept
{
  return
    +e11_*(+e22_*(e33_*e44_-e34_*e43_)-e23_*(e32_*e44_-e34_*e42_)+e24_*(e32_*e43_-e33_*e42_))
    -e21_*(+e12_*(e33_*e44_-e34_*e43_)-e13_*(e32_*e44_-e34_*e42_)+e14_*(e32_*e43_-e33_*e42_))
    +e31_*(+e12_*(e23_*e44_-e24_*e43_)-e13_*(e22_*e44_-e24_*e42_)+e14_*(e22_*e43_-e23_*e42_))
    -e41_*(+e12_*(e23_*e34_-e24_*e33_)-e13_*(e22_*e34_-e24_*e32_)+e14_*(e22_*e33_-e23_*e32_))
    ;
}

}
}
