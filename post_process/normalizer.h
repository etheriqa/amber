#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include "camera/image.h"
#include "constant.h"
#include "radiometry/rgb.h"

namespace amber {
namespace post_process {

template <typename HDR>
class Normalizer {
private:
  using hdr_image_type = camera::Image<HDR>;
  using hdr_value_type = typename HDR::value_type;

  using evaluator_type = std::function<hdr_value_type(HDR)>;

  evaluator_type evaluator_;

public:
  Normalizer() noexcept
    : evaluator_([](const auto& hdr){ return hdr.max(); }) {}

  explicit Normalizer(evaluator_type evaluator) noexcept
    : evaluator_(evaluator) {}

  hdr_image_type operator()(const hdr_image_type& image) const {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const
  {
    const auto& width = image.m_width;
    const auto& height = image.m_height;

    hdr_value_type max = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto value = evaluator_(image.pixel(i, j));
        if (std::isfinite(value)) {
          max = std::max(max, value);
        }
      }
    }

    if (max == 0) {
      return image;
    }

    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        image.pixel(i, j) /= max;
      }
    }

    return image;
  }
};

}
}
