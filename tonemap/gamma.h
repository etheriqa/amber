#pragma once

#include "image.h"
#include "rgb.h"

namespace amber {
namespace tonemap {

template <typename RealType>
class Gamma
{
public:
  using real_type      = RealType;

  using hdr_type       = RGB<real_type>;
  using ldr_type       = SRGB;

  using hdr_image_type = Image<hdr_type>;
  using ldr_image_type = Image<ldr_type>;

private:
  real_type m_gamma;

public:
  explicit Gamma() : m_gamma(2.2) {}
  explicit Gamma(real_type gamma) : m_gamma(gamma) {}

  ldr_image_type operator()(const hdr_image_type& hdr) const
  {
    const auto& width = hdr.m_width;
    const auto& height = hdr.m_height;

    ldr_image_type ldr(width, height);
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        ldr.pixel(i, j) = ldr_type(
          std::min(255.0, 256 * std::pow(hdr.pixel(i, j).x, 1 / m_gamma)),
          std::min(255.0, 256 * std::pow(hdr.pixel(i, j).y, 1 / m_gamma)),
          std::min(255.0, 256 * std::pow(hdr.pixel(i, j).z, 1 / m_gamma))
        );
      }
    }

    return ldr;
  }
};

}
}
