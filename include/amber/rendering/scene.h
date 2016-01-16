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

#include <tuple>

#include "amber/rendering/forward.h"

namespace amber {
namespace rendering {

/** Scene interface.
 */
template <typename Radiant>
class Scene
{
public:
  /** Generate a leading ray of eye path for light transport.
   */
  virtual std::tuple<ObjectPointer, Leading<Radiant>, Pixel>
  GenerateEyeRay(const Sensor& sensor, Sampler& sampler) const = 0;

  /** Generate a leading ray of light path for importance transport.
   */
  virtual std::tuple<ObjectPointer, Leading<Radiant>>
  GenerateLightRay(Sampler& sampler) const = 0;

  /** Casts a ray into the scene.
   */
  virtual std::tuple<ObjectPointer, Hit>
  Cast(const Ray& ray) const = 0;

  /** Probability that a given point on the aperture is sampled.
   */
  virtual const real_type
  EyePDFArea(const Vector3& point) const = 0;

  /** Probability that a given direction from the aperture is sampled.
   */
  virtual const real_type
  EyePDFDirection(
    const Sensor& sensor,
    const Ray& ray
  ) const = 0;

  /** Probability that a given light source is sampled.
   */
  virtual const real_type
  LightPDFArea(const ObjectPointer& object) const = 0;

  /** Probability that a given direction from the light source is sampled.
   */
  virtual const real_type
  LightPDFDirection(
    const ObjectPointer& object,
    const Ray& ray
  ) const = 0;

  /** Surface type.
   */
  virtual SurfaceType
  Surface(const ObjectPointer& object) const = 0;

  /** Emitted radiance.
   */
  virtual const Radiant
  Radiance(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out
  ) const = 0;

  /** Emitted importance.
   */
  virtual const PixelValue<Radiant>
  Response(
    const Sensor& sensor,
    const Vector3& position,
    const UnitVector3& direction_out
  ) const = 0;

  /** BSDF for light transport.
   */
  virtual const Radiant
  BSDF(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const = 0;

  /** Adjoint BSDF for importance transport.
   */
  virtual const Radiant
  AdjointBSDF(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const = 0;

  /** Sampling probability for light transport.
   */
  virtual const real_type
  PDFLight(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const = 0;

  /** Sampling probability for importance transport.
   */
  virtual const real_type
  PDFImportance(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    const UnitVector3& direction_in
  ) const = 0;

  /** Sample a scattering event for light transport.
   */
  virtual Scatter<Radiant>
  SampleLight(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const = 0;

  /** Sample a scattering event for importance transport.
   */
  virtual Scatter<Radiant>
  SampleImportance(
    const ObjectPointer& object,
    const UnitVector3& normal,
    const UnitVector3& direction_out,
    Sampler& sampler
  ) const = 0;
};

}
}
