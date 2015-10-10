#pragma once

#include "aabb.h"
#include "hit.h"
#include "random.h"
#include "ray.h"

namespace amber {
namespace primitive {

template <typename RealType>
struct Primitive
{
  using primitive_type          = Primitive<RealType>;
  using real_type               = RealType;

  using aabb_type               = AABB<real_type>;
  using hit_type                = Hit<real_type>;
  using initial_ray_sample_type = InitialRaySample<real_type>;
  using ray_type                = Ray<real_type>;

  virtual ~Primitive() {}

  virtual real_type surface_area() const noexcept = 0;
  virtual aabb_type aabb() const noexcept = 0;
  virtual hit_type intersect(const ray_type&) const noexcept = 0;
  virtual initial_ray_sample_type sample_initial_ray(Random&) const = 0;
};

}
}
