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

#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "amber/prelude/pixel.h"
#include "amber/rendering/leading.h"
#include "amber/scene/lens.h"
#include "amber/scene/material_eye.h"

namespace amber {
namespace scene {

struct BasicEyeRay
{
  Vector3 position;
  UnitVector3 normal;
  UnitVector3 direction;
  real_type weight;

  BasicEyeRay(
    const Vector3& position,
    const UnitVector3& normal,
    const UnitVector3& direction,
    real_type weight
  ) noexcept;
};

template <typename Radiant, typename BasicLensForwardee>
class BasicLensForwarder
: public Lens<Radiant>
{
public:
  template <typename... Args>
  BasicLensForwarder(Args&&... args) noexcept;

  std::vector<const Object<Radiant>*> ApertureObjects() const noexcept;

  std::tuple<const Object<Radiant>*, rendering::Leading<Radiant>, Pixel>
  GenerateRay(const rendering::Sensor& sensor, Sampler& sampler) const;

  PixelValue<Radiant>
  Response(
    const rendering::Sensor& sensor,
    const Ray& ray
  ) const noexcept;

  const real_type
  PDFArea(const Vector3& point) const noexcept;

  const real_type
  PDFDirection(const rendering::Sensor& sensor, const Ray& ray) const noexcept;

private:
  BasicLensForwardee forwardee_;
  std::unique_ptr<Material<Radiant>> eye_;
  std::unordered_map<const Primitive*, Object<Radiant>> objects_;
};





inline
BasicEyeRay::BasicEyeRay(
  const Vector3& position,
  const UnitVector3& normal,
  const UnitVector3& direction,
  real_type weight
) noexcept
: position(position)
, normal(normal)
, direction(direction)
, weight(weight)
{}

template <typename Radiant, typename BasicLensForwardee>
template <typename... Args>
BasicLensForwarder<Radiant, BasicLensForwardee>
::BasicLensForwarder(Args&&... args) noexcept
: forwardee_(args...)
, eye_(MakeEye<Radiant>())
, objects_()
{
  const auto primitives = forwardee_.AperturePrimitives();
  objects_.reserve(primitives.size());
  for (const auto& primitive : forwardee_.AperturePrimitives()) {
    objects_.emplace(primitive, Object<Radiant>(primitive, eye_.get()));
  }
}

template <typename Radiant, typename BasicLensForwardee>
std::vector<const Object<Radiant>*>
BasicLensForwarder<Radiant, BasicLensForwardee>
::ApertureObjects() const noexcept
{
  std::vector<const Object<Radiant>*> objects;
  objects.reserve(objects_.size());
  for (const auto& object : objects_) {
    objects.emplace_back(&object.second);
  }
  return objects;
}

template <typename Radiant, typename BasicLensForwardee>
std::tuple<const Object<Radiant>*, rendering::Leading<Radiant>, Pixel>
BasicLensForwarder<Radiant, BasicLensForwardee>
::GenerateRay(const rendering::Sensor& sensor, Sampler& sampler) const
{
  // TODO take a profile for finding the object
  const auto generated = forwardee_.GenerateRay(sensor, sampler);
  auto& primitive = std::get<const Primitive*>(generated);
  auto& eye_ray   = std::get<BasicEyeRay>(generated);
  auto& pixel     = std::get<Pixel>(generated);

  return std::make_tuple(
    &objects_.at(primitive),
    rendering::Leading<Radiant>(
      eye_ray.position,
      eye_ray.normal,
      eye_ray.direction,
      eye_ray.weight
    ),
    pixel
  );
}

template <typename Radiant, typename BasicLensForwardee>
PixelValue<Radiant>
BasicLensForwarder<Radiant, BasicLensForwardee>
::Response(const rendering::Sensor& sensor, const Ray& ray) const noexcept
{
  const auto response = forwardee_.Response(sensor, ray);
  return PixelValue<Radiant>(
    response.Pixel(),
    static_cast<Radiant>(response.Value())
  );
}

template <typename Radiant, typename BasicLensForwardee>
const real_type
BasicLensForwarder<Radiant, BasicLensForwardee>
::PDFArea(const Vector3& point) const noexcept
{
  return forwardee_.PDFArea(point);
}

template <typename Radiant, typename BasicLensForwardee>
const real_type
BasicLensForwarder<Radiant, BasicLensForwardee>
::PDFDirection(const rendering::Sensor& sensor, const Ray& ray) const noexcept
{
  return forwardee_.PDFDirection(sensor, ray);
}

}
}
