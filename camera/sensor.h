/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "camera/image.h"
#include "geometry/vector.h"

namespace amber {
namespace camera {

template <typename Radiant, typename RealType>
class Sensor {
public:
  using image_type   = Image<Radiant>;
  using vector3_type = geometry::Vector3<RealType>;

  RealType static constexpr kFilmSize = 0.036;

private:
  image_type* image_;
  RealType width_, height_;

public:
  explicit Sensor(image_type* image) noexcept : Sensor(image, kFilmSize) {}

  Sensor(image_type* image, RealType film_size) noexcept
    : image_(image),
      width_(film_size),
      height_(film_size / image_->width() * image_->height()) {}

  image_type* const& image() const noexcept { return image_; }

  vector3_type samplePoint(size_t x, size_t y) const noexcept {
    return vector3_type(
      - ((x + RealType(0.5)) / image_->width() - RealType(0.5)) * width_,
      ((y + RealType(0.5)) / image_->height() - RealType(0.5)) * height_,
      0
    );
  }
};

}
}
