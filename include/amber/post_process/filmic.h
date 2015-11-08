/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "camera/image.h"
#include "post_process/normalizer.h"

namespace amber {
namespace post_process {

template <typename HDR>
class Filmic {
private:
  using hdr_image_type = camera::Image<HDR>;
  using hdr_value_type = typename HDR::value_type;

  const hdr_value_type kA = 0.22; // shoulder strength
  const hdr_value_type kB = 0.30; // linear strength
  const hdr_value_type kC = 0.10; // linear angle
  const hdr_value_type kD = 0.20; // toe strength
  const hdr_value_type kE = 0.01; // toe numerator
  const hdr_value_type kF = 0.30; // toe denominator
  const hdr_value_type kW = 0.70; // linear white point
  const hdr_value_type kExposure = 16.0;

  hdr_value_type exposure_;

public:
  Filmic() noexcept : exposure_(kExposure) {}
  explicit Filmic(hdr_value_type exposure) noexcept
  : exposure_(kExposure * exposure)
  {}

  hdr_image_type operator()(const hdr_image_type& image) const noexcept {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const noexcept {
    const auto& width = image.width();
    const auto& height = image.height();

    Normalizer<HDR>()(image);

    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        auto& p = image.at(i, j);
        p = map(p * exposure_) / map(HDR(kW));
      }
    }

    return image;
  }

private:
  HDR map(const HDR& x) const noexcept {
    return (x * (x * kA + kB * kC) + kD * kE)
      / (x * (x * kA + kB) + kD * kF)
      - kE / kF;
  }
};

}
}
