/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <boost/optional.hpp>

#include "lens.h"
#include "primitive.h"
#include "sensor.h"
#include "writer.h"

namespace amber {

template <typename Radiant, typename RealType>
class Camera : public Writer {
public:
  using ray_type           = Ray<RealType>;
  using lens_type          = Lens<RealType>;
  using primitive_type     = Primitive<RealType>;
  using radiant_value_type = typename Radiant::value_type;
  using sensor_type        = Sensor<Radiant, RealType>;
  using vector3_type       = Vector3<RealType>;

private:
  sensor_type sensor_;
  lens_type const* lens_;
  primitive_type const* aperture_;
  radiant_value_type p_area_, base_weight_;
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
    p_area_(1 / sensor_.SensorArea() / aperture_->SurfaceArea()),
    base_weight_(1 / std::pow(lens_->SensorDistance(), 2) / p_area_),
    origin_(origin)
  {
    w_ = Normalize(axis);
    u_ = Normalize(Cross(up, w_));
    v_ = Normalize(Cross(w_, u_));
  }

  size_t const& imageWidth() const noexcept { return sensor_.ImageWidth(); }
  size_t const& imageHeight() const noexcept { return sensor_.ImageHeight(); }
  size_t imageSize() const noexcept { return sensor_.ImageSize(); }

  void Write(std::ostream& os) const noexcept {
    os
      << "Camera(width=" << imageWidth()
      << ", height=" << imageHeight()
      << ")" << std::endl
      << "Lens: " << *lens_;
  }

  std::tuple<ray_type, Radiant, radiant_value_type, vector3_type>
  GenerateRay(
    size_t x,
    size_t y,
    Sampler* sampler
  ) const
  {
    RealType x_sensor, y_sensor;
    std::tie(x_sensor, y_sensor) = sensor_.sampleLocalPoint(x, y, sampler);
    auto const sensor_point =
      origin_ + u_ * x_sensor + v_ * y_sensor - w_ * lens_->SensorDistance();
    auto const aperture_point = aperture_->SamplePoint(sampler).origin;
    auto const direction =
      lens_->Outgoing(sensor_point, aperture_point, origin_, w_);

    return std::make_tuple(
      ray_type(aperture_point, direction),
      Radiant(base_weight_ * std::pow(std::abs(Dot(direction, w_)), 4)),
      p_area_,
      w_
    );
  }

  boost::optional<std::tuple<size_t, size_t>>
  ResponsePoint(
    vector3_type const& direction,
    vector3_type const& aperture_point
  ) const noexcept
  {
    auto const sensor_point =
      lens_->Incoming(direction, aperture_point, origin_, w_);
    auto const det = Dot(Cross(u_, v_), w_);
    return sensor_.responsePoint(
      Dot(Cross(sensor_point, v_), w_) / det,
      Dot(Cross(u_, sensor_point), w_) / det
    );
  }
};

}
