/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include <utility>

#include <boost/optional.hpp>

namespace amber {

template <typename RealType>
boost::optional<std::tuple<RealType, RealType>>
SolveQuadratic(RealType a, RealType b, RealType c) noexcept
{
  const auto d = b * b - 4 * a * c;
  if (d < 0) {
    return boost::none;
  }

  const auto sqrt_d = std::sqrt(d);
  auto alpha = - b - sqrt_d;
  auto beta = - b + sqrt_d;
  if (std::abs(alpha) < std::abs(beta)) {
    alpha = c / beta * 2;
    beta /= 2 * a;
  } else {
    beta = c / alpha * 2;
    alpha /= 2 * a;
  }

  return std::make_tuple(alpha, beta);
}

}
