/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <initializer_list>
#include <numeric>
#include <vector>

#include "primitive/triangle.h"

namespace amber {
namespace primitive {

template <typename RealType>
class ConvexPolygon : public Primitive<RealType>
{
public:
  using aabb_type      = typename Primitive<RealType>::aabb_type;
  using hit_type       = typename Primitive<RealType>::hit_type;
  using ray_type       = typename Primitive<RealType>::ray_type;

  using vector3_type   = Vector3<RealType>;

private:
  using triangle_type  = Triangle<RealType>;

  std::vector<triangle_type> triangles_;

public:
  ConvexPolygon(std::initializer_list<vector3_type> vertices) noexcept {
    auto const it = vertices.begin();
    for (size_t i = 2; i < vertices.size(); i++) {
      triangles_.emplace_back(*std::next(it, i - 1), *std::next(it, i), *it);
    }
  }

  RealType SurfaceArea() const noexcept {
    RealType surface_area = 0;
    for (auto const& triangle : triangles_) {
      surface_area += triangle.SurfaceArea();
    }
    return surface_area;
  }

  aabb_type BoundingBox() const noexcept {
    aabb_type aabb;
    for (auto const& triangle : triangles_) {
      aabb += triangle.BoundingBox();
    }
    return aabb;
  }

  hit_type Intersect(ray_type const& ray) const noexcept {
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