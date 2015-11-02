/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "geometry/vector.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
struct Scatter {
  using radiant_value_type = typename Radiant::value_type;
  using vector3_type       = geometry::Vector3<RealType>;

  vector3_type direction_o;
  Radiant bsdf;
  radiant_value_type psa_probability;

  Scatter(vector3_type const& direction_o,
          Radiant const& bsdf,
          radiant_value_type psa_probability) noexcept
    : direction_o(direction_o),
      bsdf(bsdf),
      psa_probability(psa_probability) {}
};

}
}
