/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "vector.h"

namespace amber {

template <typename Radiant, typename RealType>
struct Scatter {
  using vector3_type       = Vector3<RealType>;

  vector3_type direction;
  Radiant weight;

  Scatter(
    vector3_type const& direction,
    Radiant const& weight
  ) noexcept
  : direction(direction),
    weight(weight)
  {}
};

}