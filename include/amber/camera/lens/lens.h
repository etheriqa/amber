/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

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

  virtual RealType sensorDistance() const noexcept = 0;

  virtual
  vector3_type
  outgoing(
    vector3_type const&, // sensor_point
    vector3_type const&, // aperture_point
    vector3_type const&, // origin
    vector3_type const&  // axis
  ) const noexcept = 0;

  virtual
  vector3_type
  incoming(
    vector3_type const&, // direction
    vector3_type const&, // aperture_point
    vector3_type const&, // origin
    vector3_type const&  // axis
  ) const noexcept = 0;
};

}
}
}
