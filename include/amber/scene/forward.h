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
#include <cstdint>

#include "amber/prelude/forward.h"
#include "amber/raytracer/forward.h"

namespace amber {
namespace scene {

using real_type = std::float_t;
using prelude::pixel_size_type;

using Vector2     = prelude::Vector2<real_type>;
using Vector3     = prelude::Vector3<real_type>;
using UnitVector3 = prelude::UnitVector3<real_type>;
using Matrix3     = prelude::Matrix3<real_type>;
using Matrix4     = prelude::Matrix4<real_type>;
using AABB        = prelude::AABB<real_type>;
using Ray         = prelude::Ray<real_type>;
using Hit         = prelude::Hit<real_type>;

using prelude::Pixel;
template <typename Radiant> using PixelValue = prelude::PixelValue<Radiant>;
template <typename Radiant> using Image      = prelude::Image<Radiant>;

using prelude::Sampler;

template <typename Radiant> class Lens;

class Primitive;
template <typename Radiant> class Material;
template <typename Radiant> class Object;
template <typename Radiant> class Scene;

template <typename Radiant>
using Acceleration = raytracer::Acceleration<real_type, Object<Radiant>>;

}
}
