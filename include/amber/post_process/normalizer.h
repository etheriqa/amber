/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <functional>

#include "constant.h"
#include "image.h"
#include "rgb.h"

namespace amber {
namespace post_process {

template <typename HDR>
class Normalizer {
private:
  using hdr_image_type = Image<HDR>;
  using hdr_value_type = typename HDR::value_type;

  using evaluator_type = std::function<hdr_value_type(HDR)>;

  evaluator_type evaluator_;

public:
  Normalizer() noexcept
    : evaluator_([](const auto& hdr){ return hdr.Max(); }) {}

  explicit Normalizer(evaluator_type evaluator) noexcept
    : evaluator_(evaluator) {}

  hdr_image_type operator()(const hdr_image_type& image) const {
    auto clone = image;
    return (*this)(clone);
  }

  hdr_image_type& operator()(hdr_image_type& image) const
  {
    const auto& width = image.width();
    const auto& height = image.height();

    hdr_value_type max = 0;
    for (size_t j = 0; j < height; j++) {
      for (size_t i = 0; i < width; i++) {
        const auto value = evaluator_(image.at(i, j));
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
        image.at(i, j) /= max;
      }
    }

    return image;
  }
};

}
}
