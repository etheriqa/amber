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
    const auto& width = hdr.m_width;
    const auto& height = hdr.m_height;

    ldr_image_type ldr(width, height);
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto& r = hdr.pixel(i, j).r();
        const auto& g = hdr.pixel(i, j).g();
        const auto& b = hdr.pixel(i, j).b();
        ldr.pixel(i, j) = LDR(
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
