/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/writer.h"
#include "camera/lens/lens.h"
#include "camera/sensor.h"
#include "geometry/first_ray.h"
#include "geometry/primitive/primitive.h"

namespace amber {
namespace camera {

template <typename Radiant, typename RealType>
class Camera : public Writer {
public:
  using first_ray_type = geometry::FirstRay<RealType>;
  using lens_type      = lens::Lens<RealType>;
  using primitive_type = geometry::primitive::Primitive<RealType>;
  using sensor_type    = Sensor<Radiant, RealType>;
  using vector3_type   = geometry::Vector3<RealType>;

private:
  sensor_type sensor_;
  lens_type const* lens_;
  primitive_type const* aperture_;
  vector3_type origin_, u_, v_, w_;

public:
  Camera(
    sensor_type const& sensor,
    lens_type const* lens,
    primitive_type const* aperture,
    vector3_type const& origin,
    vector3_type const& axis,
    vector3_type const& up
  ) noexcept
  : sensor_(sensor),
    lens_(lens),
    aperture_(aperture),
    origin_(origin)
  {
    w_ = normalize(axis);
    u_ = normalize(cross(up, w_));
    v_ = normalize(cross(w_, u_));
  }

  size_t const& imageWidth() const noexcept { return sensor_.width(); }
  size_t const& imageHeight() const noexcept { return sensor_.height(); }
  size_t imageSize() const noexcept { return imageWidth() * imageHeight(); }

  void write(std::ostream& os) const noexcept {
    os
      << "Camera(width=" << imageWidth()
      << ", height=" << imageHeight()
      << ")" << std::endl
      << "Lens: " << *lens_;
  }

  first_ray_type
  sampleFirstRay(
    size_t x,
    size_t y,
    Sampler *sampler
  ) const
  {
    RealType x_sensor, y_sensor;
    std::tie(x_sensor, y_sensor) = sensor_.sampleLocalPoint(x, y, sampler);
    const auto sensor_point =
      origin_ + u_ * x_sensor + v_ * y_sensor - w_ * lens_->sensorDistance();
    const auto aperture_point = aperture_->sampleFirstRay(sampler).origin;

    return first_ray_type(
      aperture_point,
      lens_->outgoing(sensor_point, aperture_point, origin_, w_),
      w_
    );
  }
};

}
}
