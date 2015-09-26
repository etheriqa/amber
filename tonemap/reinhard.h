#pragma once

#include <algorithm>
#include <cmath>
#include "constant.h"
#include "image.h"
#include "rgb.h"

namespace amber {
namespace tonemap {

template <typename RealType>
class Reinhard
{
public:
  using real_type      = RealType;

  using hdr_type       = RGB<real_type>;
  using ldr_type       = SRGB;

  using hdr_image_type = Image<hdr_type>;
  using ldr_image_type = Image<SRGB>;

private:
  static constexpr RealType kDelta = 1e-3;

  RealType m_key, m_gamma;

public:
  explicit Reinhard(real_type key = 0.18, real_type gamma = 2.2) :
    m_key(key), m_gamma(gamma)
  {}

  ldr_image_type operator()(const hdr_image_type& hdr) const
  {
    const auto width = hdr.m_width;
    const auto height = hdr.m_height;

    auto image = hdr;

    real_type max_luminance = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto& p = image.pixel(i, j);
        max_luminance = std::max(max_luminance, luminance(p));
      }
    }

    real_type log_sum_luminance = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        auto& p = image.pixel(i, j);
        p /= max_luminance;
        log_sum_luminance += std::log(kDelta + luminance(p));
      }
    }
    const auto log_average_luminance = std::exp(log_sum_luminance / width / height);

    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        auto& p = image.pixel(i, j);
        p *= m_key / log_average_luminance;
        p /= RGB<RealType>(1) + p;
      }
    }

    ldr_image_type ldr(width, height);
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto& p = image.pixel(i, j);
        ldr.pixel(i, j) = ldr_type(
          std::floor(255 * std::pow(p.x, 1 / m_gamma)),
          std::floor(255 * std::pow(p.y, 1 / m_gamma)),
          std::floor(255 * std::pow(p.z, 1 / m_gamma))
        );
      }
    }

    return ldr;
  }

private:
  static real_type luminance(const hdr_type& pixel)
  {
    return 0.27 * pixel.x + 0.67 * pixel.y + 0.06 * pixel.z;
  }
};

}
}
