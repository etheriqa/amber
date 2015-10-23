#pragma once

#include <sstream>
#include "camera/lens/lens.h"

namespace amber {
namespace camera {
namespace lens {

template <typename RealType>
class Pinhole : public Lens<RealType>
{
public:
  using real_type    = RealType;
  using lens_type    = Lens<real_type>;

  using ray_type     = typename lens_type::ray_type;
  using vector3_type = typename lens_type::vector3_type;

private:
  real_type m_sensor_distance;

public:
  explicit Pinhole(real_type sd = lens_type::kFocalLength) :
    m_sensor_distance(sd)
  {}

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "Lens: Pinhole(sensor_distance=" << m_sensor_distance << ")";
    return ss.str();
  }

  ray_type sample_ray(const vector3_type& sensor_point, Random& g) const
  {
    return ray_type(
      vector3_type(0, 0, 0),
      vector3_type(0, 0, -m_sensor_distance) - sensor_point
    );
  }
};

}
}
}
