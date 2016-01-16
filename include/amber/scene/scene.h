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

#include "amber/raytracer/acceleration.h"
#include "amber/rendering/object.h"
#include "amber/rendering/scene.h"
#include "amber/rendering/surface.h"
#include "amber/scene/forward.h"
#include "amber/scene/lens.h"
#include "amber/scene/light_set.h"

namespace amber {
namespace scene {

/** Concrete scene object.
 */
template <typename Radiant>
class Scene
: public rendering::Scene<Radiant>
{
public:
  /** Factory.
   */
  template <typename Acceleration>
  static Scene
  Create(
    std::vector<std::unique_ptr<Primitive>>&& primitives,
    std::vector<std::unique_ptr<Material<Radiant>>>&& materials,
    std::vector<Object<Radiant>>&& objects,
    std::unique_ptr<Lens<Radiant>>&& lens
  ) noexcept;

  /** Implementations.
   */
  std::tuple<rendering::ObjectPointer, rendering::Leading<Radiant>, Pixel>
  GenerateEyeRay(const rendering::Sensor& sensor, Sampler& sampler) const;

  std::tuple<rendering::ObjectPointer, rendering::Leading<Radiant>>
  GenerateLightRay(Sampler& sampler) const;

  std::tuple<rendering::ObjectPointer, Hit>
  Cast(const Ray& ray) const;

  const real_type
  EyePDFArea(const Vector3& point) const;

  const real_type
  EyePDFDirection(
    const rendering::Sensor& sensor,
    const Ray& ray
  ) const;

  const real_type
  LightPDFArea(const rendering::ObjectPointer& object) const;

  const real_type
  LightPDFDirection(
    const rendering::ObjectPointer& object,
    const Ray& ray
  ) const;

  rendering::SurfaceType
  Surface(const rendering::ObjectPointer& object) const;

  const Radiant
  Radiance(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out
  ) const;

  const PixelValue<Radiant>
  Response(
    const rendering::Sensor& sensor,
    const Vector3& position,
    const UnitVector3& direction_out
  ) const;

  const Radiant
  BSDF(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const;

  const Radiant
  AdjointBSDF(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const;

  const real_type
  PDFLight(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const;

  const real_type
  PDFImportance(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const;

  rendering::Scatter<Radiant>
  SampleLight(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const;

  rendering::Scatter<Radiant>
  SampleImportance(
    const rendering::ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const;

private:
  std::vector<std::unique_ptr<Primitive>> primitives_;
  std::vector<std::unique_ptr<Material<Radiant>>> materials_;
  std::unique_ptr<Acceleration<Radiant>> acceleration_;
  std::unique_ptr<Lens<Radiant>> lens_;
  LightSet<Radiant> light_set_;

  Scene(
    std::vector<std::unique_ptr<Primitive>>&& primitives,
    std::vector<std::unique_ptr<Material<Radiant>>>&& materials,
    std::unique_ptr<Acceleration<Radiant>>&& acceleration,
    std::unique_ptr<Lens<Radiant>>&& lens,
    LightSet<Radiant>&& light_set
  ) noexcept;

  static const Object<Radiant>&
  Dereference(const rendering::ObjectPointer& pointer);
};





template <typename Radiant>
template <typename Acceleration>
Scene<Radiant>
Scene<Radiant>::Create(
  std::vector<std::unique_ptr<Primitive>>&& primitives,
  std::vector<std::unique_ptr<Material<Radiant>>>&& materials,
  std::vector<Object<Radiant>>&& objects,
  std::unique_ptr<Lens<Radiant>>&& lens
) noexcept
{
  std::vector<Object<Radiant>> lights;
  for (const auto& object : objects) {
    if (object.Surface() == rendering::SurfaceType::Light) {
      lights.emplace_back(object);
    }
  }

  return Scene(
    std::move(primitives),
    std::move(materials),
    std::make_unique<Acceleration>(std::move(objects)),
    std::move(lens),
    std::move(lights)
  );
}

template <typename Radiant>
Scene<Radiant>::Scene(
  std::vector<std::unique_ptr<Primitive>>&& primitives,
  std::vector<std::unique_ptr<Material<Radiant>>>&& materials,
  std::unique_ptr<Acceleration<Radiant>>&& acceleration,
  std::unique_ptr<Lens<Radiant>>&& lens,
  LightSet<Radiant>&& light_set
) noexcept
: primitives_(std::move(primitives))
, materials_(std::move(materials))
, acceleration_(std::move(acceleration))
, lens_(std::move(lens))
, light_set_(std::move(light_set))
{}

template <typename Radiant>
std::tuple<rendering::ObjectPointer, rendering::Leading<Radiant>, Pixel>
Scene<Radiant>::GenerateEyeRay(
  const rendering::Sensor& sensor,
  Sampler& sampler
) const
{
  const auto generated = lens_->GenerateRay(sensor, sampler);
  return std::make_tuple(
    std::get<const Object<Radiant>*>(generated),
    std::get<rendering::Leading<Radiant>>(generated),
    std::get<Pixel>(generated)
  );
}

template <typename Radiant>
std::tuple<rendering::ObjectPointer, rendering::Leading<Radiant>>
Scene<Radiant>::GenerateLightRay(Sampler& sampler) const
{
  const auto generated = light_set_.GenerateRay(sampler);
  return std::make_tuple(
    std::get<const Object<Radiant>*>(generated),
    std::get<rendering::Leading<Radiant>>(generated)
  );
}

template <typename Radiant>
std::tuple<rendering::ObjectPointer, Hit>
Scene<Radiant>::Cast(const Ray& ray) const
{
  Hit hit;
  const Object<Radiant>* object;
  std::tie(hit, object) = acceleration_->Cast(ray);
  return std::make_tuple(object, hit);
}

template <typename Radiant>
const real_type
Scene<Radiant>::EyePDFArea(const Vector3& point) const
{
  return lens_->PDFArea(point);
}

template <typename Radiant>
const real_type
Scene<Radiant>::EyePDFDirection(
  const rendering::Sensor& sensor,
  const Ray& ray
) const
{
  return lens_->PDFDirection(sensor, ray);
}

template <typename Radiant>
const real_type
Scene<Radiant>::LightPDFArea(const rendering::ObjectPointer& object) const
{
  return light_set_.PDFArea(Dereference(object));
}

template <typename Radiant>
const real_type
Scene<Radiant>::LightPDFDirection(
  const rendering::ObjectPointer& object,
  const Ray& ray
) const
{
  return light_set_.PDFDirection(Dereference(object), ray);
}

template <typename Radiant>
rendering::SurfaceType
Scene<Radiant>::Surface(const rendering::ObjectPointer& object) const
{
  return Dereference(object).Surface();
}

template <typename Radiant>
const Radiant
Scene<Radiant>::Radiance(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out
) const
{
  return Dereference(object).Radiance(normal, direction_out);
}

template <typename Radiant>
const PixelValue<Radiant>
Scene<Radiant>::Response(
  const rendering::Sensor& sensor,
  const Vector3& position,
  const UnitVector3& direction_out
) const
{
  return lens_->Response(sensor, Ray(position, direction_out));
}

template <typename Radiant>
const Radiant
Scene<Radiant>::BSDF(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const
{
  return Dereference(object).BSDF(normal, direction_out, direction_in);
}

template <typename Radiant>
const Radiant
Scene<Radiant>::AdjointBSDF(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const
{
  return Dereference(object).AdjointBSDF(normal, direction_out, direction_in);
}

template <typename Radiant>
const real_type
Scene<Radiant>::PDFLight(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const
{
  return Dereference(object).PDFLight(normal, direction_out, direction_in);
}

template <typename Radiant>
const real_type
Scene<Radiant>::PDFImportance(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const
{
  return Dereference(object).PDFImportance(normal, direction_out, direction_in);
}

template <typename Radiant>
rendering::Scatter<Radiant>
Scene<Radiant>::SampleLight(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return Dereference(object).SampleLight(normal, direction_out, sampler);
}

template <typename Radiant>
rendering::Scatter<Radiant>
Scene<Radiant>::SampleImportance(
  const rendering::ObjectPointer& object,
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  return Dereference(object).SampleImportance(normal, direction_out, sampler);
}

template <typename Radiant>
const Object<Radiant>&
Scene<Radiant>::Dereference(const rendering::ObjectPointer& pointer)
{
  return static_cast<const Object<Radiant>&>(pointer);
}

}
}
