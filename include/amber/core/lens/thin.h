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

#include "core/lens.h"

namespace amber {
namespace core {
namespace lens {

template <typename RealType>
class Thin : public Lens<RealType>
{
private:
  using typename Lens<RealType>::ray_type;
  using typename Lens<RealType>::unit_vector3_type;
  using typename Lens<RealType>::vector3_type;

  RealType focal_length_, image_distance_, sensor_distance_;

public:
  explicit Thin(RealType image_distance) noexcept
  : Thin(image_distance, Lens<RealType>::kFocalLength)
  {}

  Thin(RealType image_distance, RealType focal_length) noexcept
  : focal_length_(focal_length),
    image_distance_(image_distance),
    sensor_distance_(1 / (1 / focal_length_ - 1 / image_distance_))
  {}

  RealType const SensorDistance() const noexcept { return sensor_distance_; }

  void Write(std::ostream& os) const noexcept {
    os
      << "Thin(focal_length=" << focal_length_
      << ", image_distance=" << image_distance_
      << ", sensor_distance=" << sensor_distance_
      << ")" << std::endl;
  }

  unit_vector3_type const
  Outgoing(
    vector3_type const& sensor_point,
    vector3_type const& aperture_point,
    vector3_type const& origin,
    unit_vector3_type const& axis
  ) const noexcept
  {
    auto const ratio = image_distance_ / focal_length_;
    return
      Normalize(ratio * origin + (1 - ratio) * sensor_point - aperture_point);
  }

  vector3_type const
  Incoming(
    unit_vector3_type const& direction,
    vector3_type const& aperture_point,
    vector3_type const& origin,
    unit_vector3_type const& axis
  ) const noexcept
  {
    auto const ratio = sensor_distance_ / image_distance_;
    return
      (1 + ratio) * origin -
      ratio * aperture_point -
      sensor_distance_ / Dot(direction, axis) * direction;
  }
};

}
}
}
