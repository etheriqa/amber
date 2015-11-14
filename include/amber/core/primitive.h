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

#include "core/aabb.h"
#include "core/hit.h"
#include "core/ray.h"
#include "core/sampler.h"

namespace amber {
namespace core {

template <typename RealType>
class Primitive
{
public:
  using aabb_type = AABB<RealType>;
  using hit_type  = Hit<RealType>;
  using ray_type  = Ray<RealType>;
  using real_type = RealType;

  virtual RealType SurfaceArea() const noexcept = 0;
  virtual aabb_type BoundingBox() const noexcept = 0;
  virtual hit_type Intersect(ray_type const&) const noexcept = 0;
  virtual ray_type SamplePoint(Sampler*) const = 0;
};

}
}