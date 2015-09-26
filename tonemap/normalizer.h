#pragma once

#include <algorithm>
#include <cmath>
#include <functional>
#include "constant.h"
#include "image.h"
#include "rgb.h"

namespace amber {
namespace tonemap {

template <typename RealType>
class Normalizer
{
public:
  using real_type      = RealType;

  using hdr_type       = RGB<real_type>;

  using hdr_image_type = Image<hdr_type>;

  using evaluator_type = std::function<real_type(hdr_type)>;

private:
  evaluator_type m_evaluator;

public:
  explicit Normalizer() :
    m_evaluator([](const hdr_type& hdr){ return std::max({hdr.x, hdr.y, hdr.z}); })
  {}

  explicit Normalizer(evaluator_type e) : m_evaluator(e) {}

  hdr_image_type operator()(const hdr_image_type& image) const
  {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const
  {
    const auto& width = image.m_width;
    const auto& height = image.m_height;

    real_type max = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto value = m_evaluator(image.pixel(i, j));
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
