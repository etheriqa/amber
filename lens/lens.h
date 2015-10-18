#pragma once

#include <string>
#include "random.h"
#include "geometry/ray.h"

namespace amber {
namespace lens {

template <typename RealType>
struct Lens {
  using real_type    = RealType;

  using ray_type     = geometry::Ray<real_type>;
  using vector3_type = geometry::Vector3<real_type>;

  static constexpr real_type kFocalLength = 0.050;

  virtual ~Lens() {}

  virtual std::string to_string() const = 0;
  virtual ray_type sample_ray(const vector3_type&, Random&) const = 0;
};

}
}
