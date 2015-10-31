/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "camera/lens/lens.h"

namespace amber {
namespace camera {
namespace lens {

template <typename RealType>
class Pinhole : public Lens<RealType> {
public:
  using ray_type     = typename Lens<RealType>::ray_type;
  using vector3_type = typename Lens<RealType>::vector3_type;

private:
  RealType sensor_distance_;

public:
  Pinhole() noexcept : Pinhole(Lens<RealType>::kFocalLength) {}

  explicit Pinhole(RealType sensor_distance) noexcept
    : sensor_distance_(sensor_distance) {}

  void write(std::ostream& os) const noexcept {
    os << "Pinhole(sensor_distance=" << sensor_distance_ << ")";
  }

  ray_type sampleRay(vector3_type const& sensor_point, Sampler*) const {
    return ray_type(vector3_type(0, 0, 0),
                    vector3_type(0, 0, -sensor_distance_) - sensor_point);
  }
};

}
}
}
