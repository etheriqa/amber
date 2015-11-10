/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "sampler.h"
#include "geometry/vector.h"

namespace amber {
namespace camera {

template <typename Radiant, typename RealType>
class Sensor {
public:
  using vector3_type = geometry::Vector3<RealType>;

  RealType static constexpr kFilmSize = 0.036;

private:
  size_t resolution_width_, resolution_height_;
  RealType sensor_width_, sensor_height_;

public:
  Sensor(size_t width, size_t height) noexcept
  : Sensor(width, height, kFilmSize) {}

  Sensor(size_t width, size_t height, RealType film_size) noexcept
  : resolution_width_(width),
    resolution_height_(height),
    sensor_width_(film_size),
    sensor_height_(film_size / width * height)
  {}

  size_t const& width() const noexcept { return resolution_width_; }
  size_t const& height() const noexcept { return resolution_height_; }

  std::tuple<RealType, RealType>
  sampleLocalPoint(size_t x, size_t y, Sampler*) const noexcept
  {
    // TODO random sampling
    return std::make_tuple(
      ((x + RealType(0.5)) / resolution_width_ - RealType(0.5)) *
        sensor_width_,
      ((y + RealType(0.5)) / resolution_height_ - RealType(0.5)) *
        sensor_height_
    );
  }
};

}
}
