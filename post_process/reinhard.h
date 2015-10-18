#pragma once

#include <algorithm>
#include <cmath>
#include "constant.h"
#include "image.h"
#include "radiometry/rgb.h"
#include "post_process/normalizer.h"

namespace amber {
namespace post_process {

template <typename RealType>
class Reinhard
{
public:
  using real_type      = RealType;

  using hdr_type       = radiometry::RGB<real_type>;

  using hdr_image_type = Image<hdr_type>;

private:
  static constexpr RealType kDelta = 1e-3;

  RealType m_key;

public:
  explicit Reinhard(real_type key = 0.18) :
    m_key(key)
  {}

  hdr_image_type operator()(const hdr_image_type& image) const
  {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const
  {
    const auto& width = image.m_width;
    const auto& height = image.m_height;

    Normalizer<real_type> normalizer(&luminance);
    normalizer(image);

    real_type log_sum_luminance = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto l = luminance(image.pixel(i, j));
        log_sum_luminance += std::log(kDelta + (std::isfinite(l) ? l : 0));
      }
    }
    const auto log_average_luminance = std::exp(log_sum_luminance / width / height);

    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        auto& p = image.pixel(i, j);
        p *= m_key / log_average_luminance;
        p /= hdr_type(1) + p;
      }
    }

    return image;
  }

private:
  static real_type luminance(const hdr_type& pixel)
  {
    return 0.27 * pixel.x + 0.67 * pixel.y + 0.06 * pixel.z;
  }
};

}
}
