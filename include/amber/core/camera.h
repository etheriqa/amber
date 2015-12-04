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

#include <boost/optional.hpp>

#include "core/lens.h"
#include "core/material/eye.h"
#include "core/object.h"
#include "core/primitive.h"
#include "core/sensor.h"
#include "core/writer.h"

namespace {

std::double_t const kSensitivity = 1;

}

namespace amber {
namespace core {

template <typename Radiant, typename RealType>
class Camera : public Writer
{
public:
  using eye_type           = material::Eye<Radiant, RealType>;
  using lens_type          = Lens<RealType>;
  using object_type        = Object<Radiant, RealType>;
  using primitive_type     = Primitive<RealType>;
  using radiant_value_type = typename Radiant::value_type;
  using ray_type           = Ray<RealType>;
  using sensor_type        = Sensor<Radiant, RealType>;
  using unit_vector3_type  = UnitVector3<RealType>;
  using vector3_type       = Vector3<RealType>;

private:
  sensor_type sensor_;
  lens_type const* lens_;
  object_type aperture_;
  vector3_type origin_;
  unit_vector3_type u_, v_, w_;

public:
  Camera(
    sensor_type const& sensor,
    lens_type const* lens,
    primitive_type const* aperture,
    vector3_type const& origin,
    vector3_type const& axis,
    vector3_type const& up
  ) noexcept
  : sensor_(sensor),
    lens_(lens),
    aperture_(object_type(aperture, new eye_type)),
    origin_(origin)
  {
    w_ = Normalize(axis);
    u_ = Normalize(Cross(up, w_));
    v_ = Normalize(Cross(w_, u_));
  }

  object_type const& aperture() const noexcept { return aperture_; }

  std::size_t const& imageWidth() const noexcept
  {
    return sensor_.ImageWidth();
  }

  std::size_t const& imageHeight() const noexcept
  {
    return sensor_.ImageHeight();
  }

  std::size_t const imageSize() const noexcept
  {
    return sensor_.ImageSize();
  }

  void Write(std::ostream& os) const noexcept
  {
    os
      << "Camera(width=" << imageWidth()
      << ", height=" << imageHeight()
      << ")" << std::endl
      << "Lens: " << *lens_;
  }

  radiant_value_type const
  PDFArea() const noexcept
  {
    return 1 / aperture_.SurfaceArea();
  }

  radiant_value_type const
  PDFDirection(unit_vector3_type const& direction) const noexcept
  {
    return
      imageSize() *
      std::pow(lens_->SensorDistance(), 2) /
      std::pow(std::abs(Dot(direction, w_)), 4) /
      sensor_.SensorArea();
  }

  std::tuple<
    ray_type,           // generated ray
    Radiant,            // weight
    object_type,        // eye object
    radiant_value_type, // probability in the area measure
    radiant_value_type, // probability in the projected solid angle measure
    unit_vector3_type   // normal vector
  >
  GenerateEyeRay(
    std::size_t x,
    std::size_t y,
    Sampler& sampler
  ) const
  {
    RealType x_sensor, y_sensor;
    std::tie(x_sensor, y_sensor) = sensor_.SampleLocalPoint(x, y, sampler);
    auto const sensor_point =
      origin_ + u_ * x_sensor + v_ * y_sensor - w_ * lens_->SensorDistance();
    auto const aperture_point = aperture_.SamplePoint(sampler).origin;
    auto const direction =
      lens_->Outgoing(sensor_point, aperture_point, origin_, w_);

    auto const factor =
      std::pow(
        std::abs(
          Dot(Normalize(sensor_point - aperture_point), w_) /
          Dot(direction, w_)),
        4
      );
    auto const p_area = PDFArea();
    auto const p_psa = PDFDirection(direction);

    return std::make_tuple(
      ray_type(aperture_point, direction),
      Radiant(kSensitivity * factor / p_area / p_psa),
      aperture_,
      p_area,
      p_psa,
      w_
    );
  }

  boost::optional<std::tuple<std::size_t, std::size_t, radiant_value_type>>
  Response(
    unit_vector3_type const& direction,
    vector3_type const& aperture_point
  ) const noexcept
  {
    if (Dot(direction, w_) <= 0) {
      return boost::none;
    }
    auto const sensor_point =
      lens_->Incoming(direction, aperture_point, origin_, w_);
    auto const point = sensor_.ResponsePoint(
      Dot(Cross(sensor_point, v_), w_),
      Dot(Cross(u_, sensor_point), w_)
    );
    if (!point) {
      return boost::none;
    }
    auto const factor =
      std::pow(
        std::abs(
          Dot(Normalize(sensor_point - aperture_point), w_) /
          Dot(direction, w_)),
        4
      );
    return std::tuple<std::size_t, std::size_t, radiant_value_type>(
      std::get<0>(*point),
      std::get<1>(*point),
      kSensitivity * factor
    );
  }
};

}
}
