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

#include "amber/prelude/vector3.h"

namespace amber {
namespace prelude {

template <typename T>
struct Ray
{
  Vector3<T> origin;
  UnitVector3<T> direction;

  Ray() noexcept;
  Ray(const Vector3<T>& origin, const Vector3<T>& direction) noexcept;
  Ray(const Vector3<T>& origin, const UnitVector3<T>& direction) noexcept;
};



template <typename T>
Ray<T>::Ray() noexcept
: origin()
, direction()
{}

template <typename T>
Ray<T>::Ray(const Vector3<T>& origin, const Vector3<T>& direction) noexcept
: Ray(origin, Normalize(direction))
{}

template <typename T>
Ray<T>::Ray(const Vector3<T>& origin, const UnitVector3<T>& direction) noexcept
: origin(origin)
, direction(direction)
{}

}
}
