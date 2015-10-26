/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <initializer_list>
#include <vector>
#include "geometry/primitive/primitive.h"
#include "geometry/primitive/triangle.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class ConvexPolygon : public Primitive<RealType> {
public:
  using primitive_type          = Primitive<RealType>;

  using aabb_type               = typename primitive_type::aabb_type;
  using hit_type                = typename primitive_type::hit_type;
  using initial_ray_sample_type = typename primitive_type::initial_ray_sample_type;
  using ray_type                = typename primitive_type::ray_type;
  using real_type               = typename primitive_type::real_type;

  using triangle_type           = Triangle<real_type>;
  using vector3_type            = Vector3<real_type>;

private:
  std::vector<triangle_type> m_triangles;

public:
  ConvexPolygon(std::initializer_list<vector3_type> vertices) {
    const auto it = vertices.begin();
    for (size_t i = 2; i < vertices.size(); i++) {
      m_triangles.push_back(triangle_type(
        *std::next(it, i - 1),
        *std::next(it, i),
        *it
      ));
    }
  }

  real_type surface_area() const noexcept {
    real_type area = 0;
    for (const auto& triangle : m_triangles) {
      area += triangle.surface_area();
    }
    return area;
  }

  aabb_type aabb() const noexcept {
    aabb_type aabb;
    for (const auto& triangle : m_triangles) {
      aabb += triangle.aabb();
    }
    return aabb;
  }

  hit_type intersect(const ray_type& ray) const noexcept {
    for (const auto& triangle : m_triangles) {
      const auto hit = triangle.intersect(ray);
      if (hit) {
        return hit;
      }
    }
    return hit_type();
  }

  initial_ray_sample_type sample_initial_ray(Sampler *sampler) const {
    const auto polygon_area = surface_area();
    const auto r = sampler->uniform(polygon_area);

    real_type area = 0;
    for (const auto& triangle : m_triangles) {
      const auto triangle_area = triangle.surface_area();
      area += triangle_area;
      if (area > r) {
        return triangle.sample_initial_ray(sampler);
      }
    }

    throw std::runtime_error("failed to sample initial ray");
  }
};

}
}
}
