/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <cmath>
#include "camera/image.h"
#include "post_process/normalizer.h"

namespace amber {
namespace post_process {

template <typename HDR>
class Reinhard
{
private:
  using hdr_image_type = camera::Image<HDR>;
  using hdr_value_type = typename HDR::value_type;

  const hdr_image_type kDelta = 1e-3;

  hdr_value_type key_;

public:
  Reinhard() noexcept : key_(0.18) {}
  explicit Reinhard(real_type key) noexcept : key_(key) {}

  hdr_image_type operator()(const hdr_image_type& image) const {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const {
    const auto& width = image.width();
    const auto& height = image.height();

    Normalizer<HDR>(&luminance)(image);

    real_type log_sum_luminance = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto l = luminance(image.at(i, j));
        log_sum_luminance += std::log(kDelta + (std::isfinite(l) ? l : 0));
      }
    }
    const auto log_average_luminance =
      std::exp(log_sum_luminance / width / height);

    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        auto& p = image.at(i, j);
        p *= m_key / log_average_luminance;
        p /= HDR(1) + p;
      }
    }

    return image;
  }

private:
  static hdr_value_type luminance(const HDR& pixel) noexcept
  {
    return 0.27 * pixel.r() + 0.67 * pixel.g() + 0.06 * pixel.b();
  }
};

}
}
