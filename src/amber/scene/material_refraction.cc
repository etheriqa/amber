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

#include "amber/prelude/geometry.h"
#include "amber/prelude/sampling.h"
#include "amber/rendering/surface.h"
#include "amber/scene/material_refraction.h"

namespace amber {
namespace scene {

BasicRefraction::BasicRefraction(real_type ior) noexcept
: ior_(ior)
, r0_(Fresnel(ior))
{}

rendering::SurfaceType
BasicRefraction::Surface() const noexcept
{
  return rendering::SurfaceType::Specular;
}

real_type
BasicRefraction::BSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  const auto signed_cos_alpha = Dot(direction_out, normal);
  const auto ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
  const auto squared_cos_beta =
    1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

  if (squared_cos_beta < 0) {
    // total reflection
    return kDiracDelta;
  }

  // scaling transmittance is required for the setting of light transport
  const auto rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
  const auto rho_t = (1 - rho_r) * (ior * ior);

  const auto signed_cos_i = Dot(direction_in, normal);

  if (signed_cos_alpha * signed_cos_i > 0) {
    // partial reflection
    return rho_r * kDiracDelta;
  } else {
    // refraction
    return rho_t * kDiracDelta;
  }
}

real_type
BasicRefraction::AdjointBSDF(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  const auto signed_cos_alpha = Dot(direction_out, normal);
  const auto ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
  const auto squared_cos_beta =
    1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

  if (squared_cos_beta < 0) {
    // total reflection
    return kDiracDelta;
  }

  // no scaling is required for the setting of importance transport
  const auto rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
  const auto rho_t = 1 - rho_r;

  const auto signed_cos_i = Dot(direction_in, normal);

  if (signed_cos_alpha * signed_cos_i > 0) {
    // partial reflection
    return rho_r * kDiracDelta;
  } else {
    // refraction
    return rho_t * kDiracDelta;
  }
}

real_type
BasicRefraction::PDFLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  const auto signed_cos_alpha = Dot(direction_out, normal);
  const auto ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
  const auto squared_cos_beta =
    1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

  if (squared_cos_beta < 0) {
    // total reflection
    return kDiracDelta;
  }

  // scaling transmittance is required for the setting of light transport
  const auto rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
  const auto rho_t = (1 - rho_r) * (ior * ior);
  const auto rho = rho_r + rho_t;

  const auto signed_cos_i = Dot(direction_in, normal);

  if (signed_cos_alpha * signed_cos_i > 0) {
    // partial reflection
    return
      (rho_r / rho + static_cast<real_type>(.5)) / 2 *
      static_cast<real_type>(kDiracDelta);
  } else {
    // refraction
    return
      (rho_t / rho + static_cast<real_type>(.5)) / 2 *
      static_cast<real_type>(kDiracDelta);
  }
}

real_type
BasicRefraction::PDFImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  const UnitVector3& direction_in
) const noexcept
{
  const auto signed_cos_alpha = Dot(direction_out, normal);
  const auto ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
  const auto squared_cos_beta =
    1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);

  if (squared_cos_beta < 0) {
    // total reflection
    return kDiracDelta;
  }

  // no scaling is required for the setting of importance transport
  const auto rho_r = Schlick(r0_, std::abs(signed_cos_alpha));
  const auto rho_t = 1 - rho_r;

  const auto signed_cos_i = Dot(direction_in, normal);

  if (signed_cos_alpha * signed_cos_i > 0) {
    // partial reflection
    return
      (rho_r + static_cast<real_type>(.5)) / 2 *
      static_cast<real_type>(kDiracDelta);
  } else {
    // refraction
    return
      (rho_t + static_cast<real_type>(.5)) / 2 *
      static_cast<real_type>(kDiracDelta);
  }
}

BasicScatter
BasicRefraction::SampleLight(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto signed_cos_alpha = Dot(direction_out, normal);
  const auto ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
  const auto squared_cos_beta =
    1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
  const auto direction_r =
    prelude::PerfectReflection(direction_out, normal, signed_cos_alpha);

  if (squared_cos_beta < 0) {
    // total reflection
    return BasicScatter(direction_r, 1);
  }

  const auto cos_alpha = std::abs(signed_cos_alpha);
  const auto cos_beta = std::sqrt(squared_cos_beta);
  const auto direction_t = UnitVector3(
    -ior * direction_out +
    ((signed_cos_alpha < 0 ? 1 : -1) * cos_beta + ior * signed_cos_alpha) *
    normal
  );

  // scaling transmittance is required for the setting of light transport
  const auto rho_r = Schlick(r0_, cos_alpha);
  const auto rho_t = (1 - rho_r) * (ior * ior);
  const auto rho = rho_r + rho_t;

  // sampling probabilities
  const auto p_r = (rho_r / rho + static_cast<real_type>(0.5)) / 2;
  const auto p_t = (rho_t / rho + static_cast<real_type>(0.5)) / 2;

  if (prelude::Uniform<real_type>(sampler) < p_r) {
    // partial reflection
    return BasicScatter(direction_r, rho_r / p_r);
  } else {
    // refraction
    return BasicScatter(direction_t, rho_t / p_t);
  }
}

BasicScatter
BasicRefraction::SampleImportance(
  const UnitVector3& normal,
  const UnitVector3& direction_out,
  Sampler& sampler
) const
{
  const auto signed_cos_alpha = Dot(direction_out, normal);
  const auto ior = signed_cos_alpha > 0 ? 1 / ior_ : ior_;
  const auto squared_cos_beta =
    1 - (1 - signed_cos_alpha * signed_cos_alpha) * (ior * ior);
  const auto direction_r =
    prelude::PerfectReflection(direction_out, normal, signed_cos_alpha);

  if (squared_cos_beta < 0) {
    return BasicScatter(direction_r, 1);
  }

  const auto cos_alpha = std::abs(signed_cos_alpha);
  const auto cos_beta = std::sqrt(squared_cos_beta);
  const auto direction_t = UnitVector3(
    -ior * direction_out +
    ((signed_cos_alpha < 0 ? 1 : -1) * cos_beta + ior * signed_cos_alpha) *
    normal
  );

  // no scaling is required for the setting of importance transport
  const auto rho_r = Schlick(r0_, cos_alpha);
  const auto rho_t = 1 - rho_r;

  // sampling probabilities
  const auto p_r = (rho_r + static_cast<real_type>(0.5)) / 2;
  const auto p_t = (rho_t + static_cast<real_type>(0.5)) / 2;

  if (prelude::Uniform<real_type>(sampler) < p_r) {
    // partial reflection
    return BasicScatter(direction_r, rho_r / p_r);
  } else {
    // refraction
    return BasicScatter(direction_t, rho_t / p_t);
  }
}

real_type
BasicRefraction::Fresnel(real_type ior) noexcept
{
  return std::pow((ior - 1) / (ior + 1), 2);
}

real_type
BasicRefraction::Schlick(real_type r0, real_type cos_theta) noexcept
{
  return r0 + (1 - r0) * std::pow(1 - cos_theta, 5);
}

}
}
