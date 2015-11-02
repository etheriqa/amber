/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/sampler.h"
#include "base/writer.h"
#include "geometry/ray.h"

namespace amber {
namespace camera {
namespace lens {

template <typename RealType>
class Lens : public Writer {
public:
  using ray_type     = geometry::Ray<RealType>;
  using vector3_type = geometry::Vector3<RealType>;

  RealType static constexpr kFocalLength = 0.050;

  virtual ray_type sampleRay(vector3_type const&, Sampler*) const = 0;
};

}
}
}
