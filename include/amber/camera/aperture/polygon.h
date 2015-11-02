/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
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
class Polygon : public Aperture<RealType> {
public:
  using vector3_type = typename Aperture<RealType>::vector3_type;

private:
  size_t n_;
  RealType angle_, tan_half_angle_, height_;

public:
  Polygon(size_t n, RealType height) noexcept
    : n_(n),
      angle_(2 * kPI / n),
      tan_half_angle_(std::tan(angle_ / 2)),
      height_(height) {}

  void write(std::ostream& os) const noexcept {
    os << "Polygon(n=" << n_ << ", height=" << height_ << ")";
  }

  vector3_type samplePoint(Sampler* sampler) const {
    const auto x = std::sqrt(sampler->uniform(height_ * height_));
    const auto y = x * tan_half_angle_ * sampler->uniform<RealType>(-1, 1);
    const auto angle =
      angle_ * std::floor(sampler->uniform<RealType>(n_)) +
        static_cast<RealType>(kPI) / 2;
    return vector3_type(x * std::cos(angle) - y * std::sin(angle),
                        x * std::sin(angle) + y * std::cos(angle),
                        0);
  }
};

}
}
}
