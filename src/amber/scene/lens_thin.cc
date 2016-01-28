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
#include "amber/prelude/pixel.h"
#include "amber/prelude/sampling.h"
#include "amber/rendering/sensor.h"
#include "amber/scene/lens_thin.h"
#include "amber/scene/primitive_triangle.h"

namespace amber {
namespace scene {

BasicThin::BasicThin(
  const Matrix4& transform,
  real_type focal_length,
  real_type focus_distance,
  real_type radius,
  std::size_t n_blades
) noexcept
: origin_(transform(Vector3()))
, global_(static_cast<Matrix3>(transform))
, local_(global_.Inverse())
, primitives_()
, focus_distance_(focus_distance)
, sensor_distance_(1 / (1 / focal_length - 1 / focus_distance))
, p_area_(1)
{
  for (std::size_t i = 0; i < n_blades; i++) {
    const auto alpha = 2 * static_cast<real_type>(kPI) / n_blades * i;
    const auto beta  = 2 * static_cast<real_type>(kPI) / n_blades * (i + 1);
    primitives_.emplace_back(MakeTriangle(
      origin_ + global_(radius * Vector3(std::cos(alpha), std::sin(alpha), 0)),
      origin_ + global_(radius * Vector3(std::cos(beta), std::sin(beta), 0)),
      origin_
    ));
  }
  p_area_ /= primitives_.front()->SurfaceArea() * n_blades;
}

std::vector<const Primitive*>
BasicThin::AperturePrimitives() const noexcept
{
  std::vector<const Primitive*> primitives;
  primitives.reserve(primitives_.size());
  for (const auto& primitive : primitives_) {
    primitives.emplace_back(primitive.get());
  }
  return primitives;
}

std::tuple<const Primitive*, BasicEyeRay, Pixel>
BasicThin::GenerateRay(const rendering::Sensor& sensor, Sampler& sampler) const
{
  // TODO create a integer sampling function
  const auto pos = std::min<std::size_t>(
    primitives_.size() - 1,
    std::floor(prelude::Uniform<real_type>(primitives_.size(), sampler))
  );
  const auto& primitive = primitives_[pos];

  const auto aperture_sample = primitive->SampleSurfacePoint(sampler);
  const auto aperture_point = local_(aperture_sample.Origin() - origin_);

  const auto sensor_sample = sensor.Uniform(sampler);
  const auto sensor_point =
    Vector3(std::get<Vector2>(sensor_sample), sensor_distance_);

  const auto direction = Normalize(
    - focus_distance_ / sensor_distance_ * sensor_point
    - aperture_point
  );

  const auto factor =
    std::pow(Normalize(sensor_point - aperture_point).Z() / direction.Z(), 4);

  const auto ray = Ray(aperture_sample.Origin(), global_(direction));

  return std::make_tuple(
    primitive.get(),
    BasicEyeRay(
      ray.Origin(),
      aperture_sample.Direction(),
      ray.Direction(),
      factor / PDFArea(ray.Origin()) / PDFDirection(sensor, ray)
    ),
    std::get<Pixel>(sensor_sample)
  );
}

PixelValue<real_type>
BasicThin::Response(const rendering::Sensor& sensor, const Ray& ray) const noexcept
{
  const auto direction = local_(ray.Direction());
  if (direction.Z() >= 0) {
    return PixelValue<real_type>();
  }

  const auto aperture_point = local_(ray.Origin() - origin_);
  const auto sensor_point =
    - sensor_distance_ / focus_distance_ * aperture_point
    + sensor_distance_ / direction.Z() * direction;
  const auto pixel = sensor.ResponsePixel(XY(sensor_point));
  if (!pixel) {
    return PixelValue<real_type>();
  }

  const auto factor =
    std::pow(Normalize(sensor_point - aperture_point).Z() / direction.Z(), 4);

  return PixelValue<real_type>(pixel, factor);
}

const real_type
BasicThin::PDFArea(const Vector3& point) const noexcept
{
  return p_area_;
}

const real_type
BasicThin::PDFDirection(
  const rendering::Sensor& sensor,
  const Ray& ray
) const noexcept
{
  const auto direction = local_(ray.Direction());
  return
    sensor.Size() / sensor.SceneArea() *
    std::pow(sensor_distance_, 2) / std::pow(direction.Z(), 4);
}

}
}
