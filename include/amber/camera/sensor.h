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

  size_t const& ImageWidth() const noexcept { return resolution_width_; }
  size_t const& ImageHeight() const noexcept { return resolution_height_; }

  size_t ImageSize() const noexcept
  {
    return resolution_width_ * resolution_height_;
  }

  RealType SensorArea() const noexcept
  {
    return sensor_width_ * sensor_height_ / ImageSize();
  }

  std::tuple<RealType, RealType>
  sampleLocalPoint(size_t x, size_t y, Sampler* sampler) const noexcept
  {
    auto const x_normalized =
      (x + sampler->uniform<RealType>()) / resolution_width_;
    auto const y_normalized =
      (y + sampler->uniform<RealType>()) / resolution_height_;
    return std::make_tuple(
      (x_normalized - RealType(0.5)) * sensor_width_,
      (y_normalized - RealType(0.5)) * sensor_height_
    );
  }

  boost::optional<std::tuple<size_t, size_t>>
  responsePoint(RealType x, RealType y) const noexcept
  {
    size_t const x_image =
      std::floor((x / sensor_width_ + RealType(0.5)) * resolution_width_);
    size_t const y_image =
      std::floor((y / sensor_height_ + RealType(0.5)) * resolution_height_);
    if (x_image >= resolution_width_ || y_image >= resolution_height_) {
      return boost::none;
    } else {
      return std::make_tuple(x_image, y_image);
    }
  }
};

}
}
