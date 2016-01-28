// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include "amber/constants.h"
#include "amber/prelude/matrix4.h"
#include "amber/rendering/sensor.h"
#include "amber/scene/lens_pinhole.h"
#include "amber/scene/primitive_triangle.h"

namespace amber {
namespace scene {

BasicPinhole::BasicPinhole(
  const Matrix4& transform,
  real_type sensor_distance
) noexcept
: origin_(transform(Vector3()))
, global_(static_cast<Matrix3>(transform))
, local_(global_.Inverse())
, pinhole_(MakeTriangle(origin_, origin_, origin_))
, sensor_distance_(sensor_distance)
{
}

std::vector<const Primitive*>
BasicPinhole::AperturePrimitives() const noexcept
{
  return {pinhole_.get()};
}

std::tuple<const Primitive*, BasicEyeRay, Pixel>
BasicPinhole::GenerateRay(
  const rendering::Sensor& sensor,
  Sampler& sampler
) const
{
  const auto sensor_sample = sensor.Uniform(sampler);
  const auto sensor_point =
    Vector3(std::get<Vector2>(sensor_sample), sensor_distance_);
  const auto ray = Ray(origin_, global_(-sensor_point));

  return std::make_tuple(
    pinhole_.get(),
    BasicEyeRay(
      ray.Origin(),
      Normalize(global_(Vector3(0, 0, -1))),
      ray.Direction(),
      1 / PDFArea(Vector3()) / PDFDirection(sensor, ray)
    ),
    std::get<Pixel>(sensor_sample)
  );
}

PixelValue<real_type>
BasicPinhole::Response(
  const rendering::Sensor& sensor,
  const Ray& ray
) const noexcept
{
  const auto direction = local_(ray.Direction());
  const auto point = sensor_distance_ / direction.Z() * direction;

  const auto response = sensor.ResponsePixel(XY(point));
  if (!response) {
    return PixelValue<real_type>();
  }
  return PixelValue<real_type>(response, 1);
}

const real_type
BasicPinhole::PDFArea(const Vector3& point) const noexcept
{
  return kDiracDelta;
}

const real_type
BasicPinhole::PDFDirection(
  const rendering::Sensor& sensor,
  const Ray& ray
) const noexcept
{
  const auto direction = local_(ray.Direction());
  const auto point = sensor_distance_ / direction.Z() * direction;

  const auto p_area = 1 / sensor.SceneArea();
  const auto geometry_factor =
    direction.Z() * direction.Z() / SquaredLength(point);
  return p_area / geometry_factor;
}

}
}
