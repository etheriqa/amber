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

namespace amber {
namespace camera {
namespace aperture {

template <typename RealType>
struct Aperture
{
  using aperture_type = Aperture<RealType>;
  using real_type     = RealType;

  using vector3_type  = geometry::Vector3<real_type>;

  virtual std::string to_string() const = 0;
  virtual vector3_type sample_point(Sampler*) const = 0;
};

}
}
}
