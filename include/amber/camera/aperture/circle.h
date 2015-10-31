/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>

#include "base/constant.h"
#include "camera/aperture/aperture.h"

namespace amber {
namespace camera {
namespace aperture {

template <typename RealType>
class Circle : public Aperture<RealType> {
public:
  using vector3_type = typename Aperture<RealType>::vector3_type;

private:
  RealType radius_;

public:
  explicit Circle(RealType radius) noexcept : radius_(radius) {}

  void write(std::ostream& os) const noexcept {
    os << "Circle(radius=" << radius_ << ")";
  }

  vector3_type samplePoint(Sampler* sampler) const {
    const auto radius = std::sqrt(sampler->uniform(radius_ * radius_));
    const auto theta = sampler->uniform<RealType>(2 * kPI);
    return vector3_type(radius * std::cos(theta),
                        radius * std::cos(theta),
                        0);
  }
};

}
}
}
