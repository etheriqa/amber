/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "lens.h"

namespace amber {
namespace lens {

template <typename RealType>
class Thin : public Lens<RealType>
{
public:
  using ray_type      = typename Lens<RealType>::ray_type;
  using vector3_type  = typename Lens<RealType>::vector3_type;

private:
  RealType focal_length_, focus_distance_, sensor_distance_;

public:
  explicit Thin(RealType focus_distance) noexcept
  : Thin(focus_distance, Lens<RealType>::kFocalLength)
  {}

  Thin(RealType focus_distance, RealType focal_length) noexcept
  : focal_length_(focal_length),
    focus_distance_(focus_distance),
    sensor_distance_(1 / (1 / focal_length_ - 1 / focus_distance_))
  {}

  RealType SensorDistance() const noexcept { return sensor_distance_; }

  void Write(std::ostream& os) const noexcept {
    os
      << "Thin(focal_length=" << focal_length_
      << ", focus_distance=" << focus_distance_
      << ", sensor_distance=" << sensor_distance_
      << ")" << std::endl;
  }

  vector3_type
  Outgoing(
    vector3_type const& sensor_point,
    vector3_type const& aperture_point,
    vector3_type const& origin,
    vector3_type const& axis
  ) const noexcept
  {
    auto const ratio = focus_distance_ / focal_length_;
    return
      Normalize(ratio * origin + (1 - ratio) * sensor_point - aperture_point);
  }

  vector3_type
  Incoming(
    vector3_type const& direction,
    vector3_type const& aperture_point,
    vector3_type const& origin,
    vector3_type const& axis
  ) const noexcept
  {
    auto const ratio = sensor_distance_ / focus_distance_;
    return
      (1 + ratio) * origin -
      ratio * aperture_point -
      sensor_distance_ / Dot(direction, axis) * direction;
  }
};

}
}
