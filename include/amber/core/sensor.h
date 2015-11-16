// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#pragma once

#include "core/sampler.h"
#include "core/vector3.h"

namespace amber {
namespace core {

template <typename Radiant, typename RealType>
class Sensor {
public:
  using vector3_type = Vector3<RealType>;

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
  sampleLocalPoint(size_t x, size_t y, Sampler& sampler) const noexcept
  {
    auto const x_normalized =
      (x + Uniform<RealType>(sampler)) / resolution_width_;
    auto const y_normalized =
      (y + Uniform<RealType>(sampler)) / resolution_height_;
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
