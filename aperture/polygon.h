#pragma once

#include <cmath>
#include <sstream>
#include "aperture/aperture.h"
#include "constant.h"

namespace amber {
namespace aperture {

template <typename RealType>
class Polygon : public Aperture<RealType>
{
public:
  using aperture_type = Aperture<RealType>;

  using real_type     = typename aperture_type::real_type;
  using vector3_type  = typename aperture_type::vector3_type;

private:
  size_t    m_n;
  real_type m_angle,
            m_tan_half_angle,
            m_height;

public:
  Polygon(size_t n, real_type height) :
    m_n(n),
    m_angle(2 * static_cast<real_type>(kPI) / m_n),
    m_tan_half_angle(std::tan(m_angle / 2)),
    m_height(height)
  {}

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "Aperture: Polygon(n=" << m_n << ", height=" << m_height << ")";
    return ss.str();
  }

  vector3_type sample_point(Random& g) const
  {
    const auto x = std::sqrt(g.uniform(m_height * m_height));
    const auto y = x * m_tan_half_angle * g.uniform<real_type>(-1, 1);
    const auto angle = m_angle * std::floor(g.uniform<real_type>(m_n)) + static_cast<real_type>(kPI) / 2;

    return vector3_type(
      x * std::cos(angle) - y * std::sin(angle),
      x * std::sin(angle) + y * std::cos(angle),
      0
    );
  }
};

}
}
