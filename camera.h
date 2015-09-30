#pragma once

#include <cmath>
#include <sstream>
#include <string>
#include "lens/lens.h"
#include "random.h"
#include "ray.h"
#include "sensor.h"
#include "vector.h"

namespace amber {

template <class Flux>
class Camera
{
public:
  using flux_type               = Flux;

  using real_type               = typename flux_type::real_type;

  using initial_ray_sample_type = InitialRaySample<real_type>;
  using lens_reference          = lens::Lens<real_type>*;
  using ray_type                = Ray<real_type>;
  using sensor_reference        = Sensor<flux_type>*;
  using vector3_type            = Vector3<real_type>;

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

  initial_ray_sample_type sample_initial_ray(size_t x, size_t y, Random& g) const
  {
    const auto sensor_point = m_sensor->sample_point(x, y);
    const auto ray = m_lens->sample_ray(sensor_point, g);

    return initial_ray_sample_type(
      ray_type(
        m_origin + ray.origin.x * m_u + ray.origin.y * m_v + ray.origin.z * m_w,
        ray.direction.x * m_u + ray.direction.y * m_v + ray.direction.z * m_w
      ),
      m_w
    );
  }

  void expose(size_t x, size_t y, const flux_type& flux) const
  {
    m_sensor->expose(x, y, flux);
  }
};

}
