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

#include <initializer_list>
#include <numeric>
#include <vector>

#include "core/primitive/triangle.h"

namespace amber {
namespace core {
namespace primitive {

template <typename RealType>
class ConvexPolygon : public Primitive<RealType>
{
public:
  using vector3_type = Vector3<RealType>;

private:
  using typename Primitive<RealType>::aabb_type;
  using typename Primitive<RealType>::hit_type;
  using typename Primitive<RealType>::ray_type;

  using triangle_type = Triangle<RealType>;

  std::vector<triangle_type> triangles_;

public:
  ConvexPolygon(std::initializer_list<vector3_type> const& vertices) noexcept
  {
    auto const it = vertices.begin();
    for (size_t i = 2; i < vertices.size(); i++) {
      triangles_.emplace_back(*std::next(it, i - 1), *std::next(it, i), *it);
    }
  }

  RealType SurfaceArea() const noexcept
  {
    RealType surface_area = 0;
    for (auto const& triangle : triangles_) {
      surface_area += triangle.SurfaceArea();
    }
    return surface_area;
  }

  aabb_type BoundingBox() const noexcept
  {
    aabb_type aabb;
    for (auto const& triangle : triangles_) {
      aabb += triangle.BoundingBox();
    }
    return aabb;
  }

  hit_type Intersect(ray_type const& ray) const noexcept
  {
    for (auto const& triangle : triangles_) {
      auto const hit = triangle.Intersect(ray);
      if (hit) {
        return hit;
      }
    }
    return hit_type();
  }

  ray_type SamplePoint(Sampler* sampler) const
  {
    auto const polygon_area = SurfaceArea();
    auto const r = sampler->uniform(polygon_area);

    RealType area = 0;
    for (auto const& triangle : triangles_) {
      auto const triangle_area = triangle.SurfaceArea();
      area += triangle_area;
      if (area > r) {
        return triangle.SamplePoint(sampler);
      }
    }

    throw std::runtime_error("failed to sample initial ray");
  }
};

}
}
}
