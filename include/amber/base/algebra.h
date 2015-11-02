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

namespace amber {

template <typename RealType>
std::tuple<bool, RealType, RealType> solve_quadratic(RealType a, RealType b, RealType c) noexcept
{
  const auto D = b * b - 4 * a * c;
  if (D < 0) {
    return std::make_tuple(false, RealType(), RealType());
  }

  const auto sqrt_D = std::sqrt(D);
  auto alpha = - b - sqrt_D;
  auto beta = - b + sqrt_D;
  if (std::abs(alpha) < std::abs(beta)) {
    alpha = c / beta * 2;
    beta /= 2 * a;
  } else {
    beta = c / alpha * 2;
    alpha /= 2 * a;
  }

  return std::make_tuple(true, alpha, beta);
}

}
