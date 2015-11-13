/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "writer.h"
#include "ray.h"

namespace amber {

template <typename RealType>
class Lens : public Writer
{
public:
  using ray_type     = Ray<RealType>;
  using vector3_type = Vector3<RealType>;

  RealType static constexpr kFocalLength = 0.050;

  virtual RealType SensorDistance() const noexcept = 0;

  virtual
  vector3_type // direction
  Outgoing(
    vector3_type const&, // sensor_point
    vector3_type const&, // aperture_point
    vector3_type const&, // origin
    vector3_type const&  // axis
  ) const noexcept = 0;

  virtual
  vector3_type // sensor_point
  Incoming(
    vector3_type const&, // direction
    vector3_type const&, // aperture_point
    vector3_type const&, // origin
    vector3_type const&  // axis
  ) const noexcept = 0;
};

}
