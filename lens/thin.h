#pragma once

#include <cmath>
#include <sstream>
#include "aperture/aperture.h"
#include "constant.h"
#include "lens/lens.h"

namespace amber {
namespace lens {

template <typename RealType>
class ThinLens : public Lens<RealType>
{
public:
  using real_type          = RealType;
  using lens_type          = Lens<real_type>;

  using aperture_reference = aperture::Aperture<real_type>*;
  using ray_type           = geometry::Ray<real_type>;
  using vector3_type       = geometry::Vector3<real_type>;

private:
  aperture_reference m_aperture;
  real_type          m_focal_length,
                     m_target_distance,
                     m_sensor_distance,
                     m_magnifier;

public:
  explicit ThinLens(aperture_reference a, real_type td, real_type fl = lens_type::kFocalLength) :
    m_aperture(a),
    m_focal_length(fl),
    m_target_distance(td),
    m_sensor_distance(1 / (1 / m_focal_length - 1 / m_target_distance)),
    m_magnifier(m_target_distance / m_sensor_distance)
  {}

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "Lens: ThinLens(focal_length=" << m_focal_length << ", target_distance=" << m_target_distance << ", sensor_distance=" << m_sensor_distance << ")" << std::endl;
    ss << m_aperture->to_string();
    return ss.str();
  }

  ray_type sample_ray(const vector3_type& sensor_point, Random& g) const
  {
    const auto lens_point = m_aperture->sample_point(g);
    const auto focal_point = (vector3_type(0, 0, -m_sensor_distance) - sensor_point) * m_magnifier;

    return ray_type(lens_point, focal_point - lens_point);
  }
};

}
}
