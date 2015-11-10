/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
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
class Thin : public Lens<RealType> {
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

  RealType sensorDistance() const noexcept { return sensor_distance_; }

  void write(std::ostream& os) const noexcept {
    os
      << "Thin(focal_length=" << focal_length_
      << ", focus_distance=" << focus_distance_
      << ", sensor_distance=" << sensor_distance_
      << ")" << std::endl;
  }

  vector3_type
  outgoing(
    vector3_type const& sensor_point,
    vector3_type const& aperture_point,
    vector3_type const& origin,
    vector3_type const&
  ) const noexcept
  {
    auto const ratio = focus_distance_ / focal_length_;
    return
      normalize(ratio * origin + (1 - ratio) * sensor_point - aperture_point);
  }

  vector3_type
  incoming(
    vector3_type const& direction,
    vector3_type const& aperture_point,
    vector3_type const& origin,
    vector3_type const& axis
  ) const noexcept
  {
    throw std::logic_error("Thin::incoming(): not yet implemented");
  }
};

}
}
}
