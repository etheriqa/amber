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
#include "camera/aperture/aperture.h"
#include "constant.h"

namespace amber {
namespace camera {
namespace aperture {

template <typename RealType>
class Circle : public Aperture<RealType>
{
public:
  using aperture_type = Aperture<RealType>;

  using real_type     = typename aperture_type::real_type;
  using vector3_type  = typename aperture_type::vector3_type;

private:
  real_type m_radius;

public:
  explicit Circle(real_type r) : m_radius(r) {}

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "Aperture: Circle(radius=" << m_radius << ")";
    return ss.str();
  }

  vector3_type sample_point(Random& g) const
  {
    // TODO refactor
    const auto radius = std::sqrt(g.uniform(m_radius * m_radius));
    const auto theta = g.uniform(2 * static_cast<real_type>(kPI));
    return vector3_type(radius * std::cos(theta), radius * std::cos(theta), 0);
  }
};

}
}
}
