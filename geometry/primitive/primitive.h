#pragma once

#include "geometry/aabb.h"
#include "geometry/hit.h"
#include "geometry/ray.h"
#include "random.h"

namespace amber {
namespace geometry {
namespace primitive {

template <typename RealType>
class Primitive {
public:
  using aabb_type               = AABB<RealType>;
  using hit_type                = Hit<RealType>;
  using initial_ray_sample_type = InitialRaySample<RealType>;
  using primitive_type          = Primitive<RealType>;
  using ray_type                = Ray<RealType>;
  using real_type               = RealType;

  virtual real_type surface_area() const noexcept = 0;
  virtual aabb_type aabb() const noexcept = 0;
  virtual hit_type intersect(const ray_type&) const noexcept = 0;
  virtual initial_ray_sample_type sample_initial_ray(Random&) const = 0;
};

}
}
}
