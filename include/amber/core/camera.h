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

#include "core/initial_ray.h"
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
  using sensor_type        = Sensor<RealType>;
  using unit_vector3_type  = UnitVector3<RealType>;
  using vector3_type       = Vector3<RealType>;

  using initial_ray_type    = InitialRay<object_type>;
  using initial_ray_uv_type = InitialRayUV<object_type>;

private:
  sensor_type sensor_;
  lens_type const* lens_;
  object_type aperture_;
  vector3_type origin_;
  unit_vector3_type u_, v_, w_;

public:
  Camera(
    sensor_type const& sensor,
    lens_type const* const lens,
    primitive_type const* const aperture,
    vector3_type const& origin,
    vector3_type const& axis,
    vector3_type const& up
  ) noexcept
  : sensor_(sensor)
  , lens_(lens)
  , aperture_(object_type(aperture, new eye_type)) // XXX ugly
  , origin_(origin)
  , u_(Normalize(Cross(up, axis)))
  , v_(Normalize(Cross(axis, u_)))
  , w_(Normalize(axis))
  {}

  object_type const& aperture() const noexcept { return aperture_; } // XXX ugly

  std::size_t const ImageWidth() const noexcept;
  std::size_t const ImageHeight() const noexcept;
  std::size_t const ImageSize() const noexcept;

  std::size_t const
  PackUV(std::size_t const u, std::size_t const v) const noexcept
  {
    // XXX ugly
    // TODO check the range of the input values
    return u + v * ImageWidth();
  }

  std::tuple<std::size_t, std::size_t>
  UnpackUV(std::size_t const i) const noexcept
  {
    // XXX ugly
    // TODO check the range of the input value
    return std::make_tuple(i % ImageWidth(), i / ImageWidth());
  }

  void Write(std::ostream& os) const noexcept;

  radiant_value_type const
  PDFArea() const noexcept;

  radiant_value_type const
  PDFDirection(unit_vector3_type const& direction) const noexcept;

  initial_ray_uv_type
  GenerateEyeRay(Sampler& sampler) const;

  initial_ray_type
  GenerateEyeRay(
    std::size_t const u,
    std::size_t const v,
    Sampler& sampler
  ) const;

  boost::optional<std::tuple<std::size_t, std::size_t, radiant_value_type>>
  Response(
    unit_vector3_type const& direction,
    vector3_type const& aperture_point
  ) const noexcept;

private:
  initial_ray_type
  GenerateEyeRay(
    RealType const u_sensor,
    RealType const v_sensor,
    Sampler& sampler
  ) const;
};

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::ImageWidth() const noexcept
-> std::size_t const
{
  return sensor_.ImageWidth();
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::ImageHeight() const noexcept
-> std::size_t const
{
  return sensor_.ImageHeight();
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::ImageSize() const noexcept
-> std::size_t const
{
  return sensor_.ImageSize();
}

template <typename Radiant, typename RealType>
void
Camera<Radiant, RealType>::Write(std::ostream& os) const noexcept
{
  os
    << "Camera(width=" << ImageWidth()
    << ", height=" << ImageHeight()
    << ")" << std::endl
    << "Lens: " << *lens_;
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::PDFArea() const noexcept
-> radiant_value_type const
{
  return 1 / aperture_.SurfaceArea();
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::PDFDirection(
  unit_vector3_type const& direction
) const noexcept
-> radiant_value_type const
{
  return
    ImageSize() *
    std::pow(lens_->SensorDistance(), 2) /
    std::pow(std::abs(Dot(direction, w_)), 4) /
    sensor_.SensorArea();
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::GenerateEyeRay(
  Sampler& sampler
) const
-> initial_ray_uv_type
{
  RealType u_sensor, v_sensor;
  std::size_t u, v;
  std::tie(u_sensor, v_sensor, u, v) = sensor_.SampleLocalPoint(sampler);

  return std::tuple_cat(
    GenerateEyeRay(u_sensor, v_sensor, sampler),
    std::make_tuple(u, v)
  );
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::GenerateEyeRay(
  std::size_t const u,
  std::size_t const v,
  Sampler& sampler
) const
-> initial_ray_type
{
  RealType u_sensor, v_sensor;
  std::tie(u_sensor, v_sensor) = sensor_.SampleLocalPoint(u, v, sampler);

  return GenerateEyeRay(u_sensor, v_sensor, sampler);
}

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::GenerateEyeRay(
  RealType const u_sensor,
  RealType const v_sensor,
  Sampler& sampler
) const
-> initial_ray_type
{
  auto const sensor_point =
    origin_ + u_ * u_sensor + v_ * v_sensor - w_ * lens_->SensorDistance();
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

template <typename Radiant, typename RealType>
auto
Camera<Radiant, RealType>::Response(
  unit_vector3_type const& direction,
  vector3_type const& aperture_point
) const noexcept
-> boost::optional<std::tuple<std::size_t, std::size_t, radiant_value_type>>
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

}
}
