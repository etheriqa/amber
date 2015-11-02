/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "camera/image.h"
#include "radiometry/srgb.h"

namespace amber {
namespace post_process {

template <typename HDR, typename LDR = radiometry::SRGB>
class Gamma {
private:
  using hdr_image_type = camera::Image<HDR>;
  using hdr_value_type = typename HDR::value_type;
  using ldr_image_type = camera::Image<LDR>;
  using ldr_value_type = typename LDR::value_type;

private:
  hdr_value_type gamma_;

public:
  Gamma() noexcept : gamma_(2.2) {}
  explicit Gamma(hdr_value_type gamma) noexcept : gamma_(gamma) {}

  ldr_image_type operator()(const hdr_image_type& hdr) const {
    const auto& width = hdr.width();
    const auto& height = hdr.height();

    ldr_image_type ldr(width, height);
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto& r = hdr.at(i, j).r();
        const auto& g = hdr.at(i, j).g();
        const auto& b = hdr.at(i, j).b();
        ldr.at(i, j) = LDR(
          std::min<hdr_value_type>(255, 255 * std::pow(r, 1 / gamma_)),
          std::min<hdr_value_type>(255, 255 * std::pow(g, 1 / gamma_)),
          std::min<hdr_value_type>(255, 255 * std::pow(b, 1 / gamma_))
        );
      }
    }

    return ldr;
  }
};

}
}
