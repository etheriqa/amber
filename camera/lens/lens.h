/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <string>
#include "base/sampler.h"
#include "geometry/ray.h"

namespace amber {
namespace camera {
namespace lens {

template <typename RealType>
struct Lens {
  using real_type    = RealType;

  using ray_type     = geometry::Ray<real_type>;
  using vector3_type = geometry::Vector3<real_type>;

  static constexpr real_type kFocalLength = 0.050;

  virtual ~Lens() {}

  virtual std::string to_string() const = 0;
  virtual ray_type sample_ray(const vector3_type&, Sampler*) const = 0;
};

}
}
}
