/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include "camera/lens/lens.h"
#include "camera/sensor.h"

namespace amber {
namespace camera {

template <typename Radiant, typename RealType>
class Camera
{
public:
  using radiant_type            = Radiant;
  using real_type               = RealType;

  using initial_ray_sample_type = geometry::InitialRaySample<real_type>;
  using lens_reference          = lens::Lens<real_type>*;
  using ray_type                = geometry::Ray<real_type>;
  using sensor_reference        = Sensor<radiant_type, real_type>*;
  using vector3_type            = geometry::Vector3<real_type>;

private:
  lens_reference m_lens;
  sensor_reference m_sensor;
  vector3_type m_origin, m_u, m_v, m_w;

public:
  Camera(lens_reference l, sensor_reference s, const vector3_type& origin, const vector3_type& eye, const vector3_type& up) :
    m_lens(l),
    m_sensor(s),
    m_origin(origin)
  {
    m_w = normalize(origin - eye);
    m_u = normalize(cross(up, m_w));
    m_v = normalize(cross(m_w, m_u));
  }

  size_t image_width() const
  {
    return m_sensor->image_width();
  }

  size_t image_height() const
  {
    return m_sensor->image_height();
  }

  size_t image_pixels() const
  {
    return m_sensor->image_pixels();
  }

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "Camera: Camera(width=" << image_width() << ", height=" << image_height() << ")" << std::endl;
    ss << m_lens->to_string();
    return ss.str();
  }

  initial_ray_sample_type sample_initial_ray(size_t x, size_t y, Sampler *sampler) const
  {
    const auto sensor_point = m_sensor->sample_point(x, y);
    const auto ray = m_lens->sample_ray(sensor_point, sampler);

    return initial_ray_sample_type(
      ray_type(
        m_origin + ray.origin.x() * m_u + ray.origin.y() * m_v + ray.origin.z() * m_w,
        ray.direction.x() * m_u + ray.direction.y() * m_v + ray.direction.z() * m_w
      ),
      m_w
    );
  }

  void expose(size_t x, size_t y, const radiant_type& power) const
  {
    m_sensor->expose(x, y, power);
  }
};

}
}
