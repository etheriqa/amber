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

#include <cstdint>

namespace amber {
namespace prelude {

using pixel_size_type = std::uint_fast32_t;

template <typename T> class Vector1;
template <typename T> class Vector2;
template <typename T> class Vector3;
template <typename T> class UnitVector3;
template <typename T> class Matrix3;
template <typename T> class Matrix4;
template <typename T> class AABB;
template <typename T> struct Ray;
template <typename T> struct Hit;

class Pixel;
template <typename T> class PixelValue;
template <typename T> class Image;
template <typename T> class SparseImage;

class Sampler;
template <typename Engine> class GenericSampler;

template <typename T> class Accumulator;
template <typename T, typename U> class KDTree;

enum class Axis;

}
}
