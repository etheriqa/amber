/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
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

namespace amber {
namespace camera {

template <typename Radiant, typename RealType>
class Camera : public Writer {
public:
  using first_ray_type = geometry::FirstRay<RealType>;
  using lens_type      = lens::Lens<RealType>;
  using sensor_type    = Sensor<Radiant, RealType>;
  using vector3_type   = geometry::Vector3<RealType>;

private:
  sensor_type sensor_;
  lens_type* lens_;
  vector3_type origin_, u_, v_, w_;

public:
  Camera(sensor_type sensor,
         lens_type* lens,
         vector3_type const& origin,
         vector3_type const& eye,
         vector3_type const& up) noexcept
    : sensor_(sensor), lens_(lens), origin_(origin) {
    w_ = normalize(origin - eye);
    u_ = normalize(cross(up, w_));
    v_ = normalize(cross(w_, u_));
  }

  size_t const& imageWidth() const noexcept {
    return sensor_.image()->width();
  }

  size_t const& imageHeight() const noexcept {
    return sensor_.image()->height();
  }

  size_t imageSize() const noexcept {
    return imageWidth() * imageHeight();
  }

  void write(std::ostream& os) const noexcept {
    os
      << "Camera(width=" << imageWidth()
      << ", height=" << imageHeight()
      << ")" << std::endl
      << "Lens: " << *lens_;
  }

  first_ray_type
  sampleFirstRay(size_t x,
                     size_t y,
                     Sampler *sampler) const {
    const auto sensor_point = sensor_.samplePoint(x, y);
    const auto ray = lens_->sampleRay(sensor_point, sampler);

    return first_ray_type(
      origin_ +
        ray.origin.x() * u_ +
        ray.origin.y() * v_ +
        ray.origin.z() * w_,
      ray.direction.x() * u_ +
        ray.direction.y() * v_ +
        ray.direction.z() * w_,
      w_
    );
  }

  void expose(size_t x, size_t y, const Radiant& power) const {
    sensor_.image()->at(x, y) += power;
  }
};

}
}
