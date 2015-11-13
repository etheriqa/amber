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

#include <functional>

#include "surface_type.h"

namespace amber {

template <typename Primitive, typename Material>
class Object {
public:
  using primitive_type     = Primitive;
  using material_type      = Material;

  using aabb_type          = typename primitive_type::aabb_type;
  using hit_type           = typename primitive_type::hit_type;
  using ray_type           = typename primitive_type::ray_type;
  using real_type          = typename primitive_type::real_type;

  using radiant_type       = typename material_type::radiant_type;
  using radiant_value_type = typename material_type::radiant_value_type;
  using scatter_type       = typename material_type::scatter_type;
  using vector3_type       = typename material_type::vector3_type;

  struct Hash {
    size_t operator()(Object const& o) const noexcept {
      return
        std::hash<size_t>()(reinterpret_cast<size_t>(o.primitive_)) +
        std::hash<size_t>()(reinterpret_cast<size_t>(o.material_));
    }
  };

  struct EqualTo {
    bool operator()(Object const& a, Object const& b) const noexcept {
      return a == b;
    }
  };

private:
  primitive_type const* primitive_;
  material_type const* material_;

public:
  Object() noexcept : primitive_(nullptr), material_(nullptr) {}

  Object(
    primitive_type const* primitive,
    material_type const* material
  ) noexcept
  : primitive_(primitive), material_(material) {}

  operator bool() const noexcept {
    return primitive_ != nullptr && material_ != nullptr;
  }

  bool operator==(Object const& o) const noexcept {
    return primitive_ == o.primitive_ && material_ == o.material_;
  }

  bool operator!=(Object const& o) const noexcept {
    return !(*this == o);
  }

  real_type SurfaceArea() const noexcept {
    return primitive_->SurfaceArea();
  }

  aabb_type BoundingBox() const noexcept {
    return primitive_->BoundingBox();
  }

  hit_type Intersect(ray_type const& ray) const noexcept {
    return primitive_->Intersect(ray);
  }

  ray_type SamplePoint(Sampler* sampler) const {
    return primitive_->SamplePoint(sampler);
  }

  SurfaceType Surface() const noexcept {
    return material_->Surface();
  }

  radiant_type Radiance() const noexcept {
    return material_->Radiance();
  }

  radiant_type
  BSDF(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return material_->BSDF(direction_i, direction_o, normal);
  }

  radiant_value_type
  PDFLight(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return material_->PDFLight(direction_i, direction_o, normal);
  }

  radiant_value_type
  PDFImportance(
    vector3_type const& direction_i,
    vector3_type const& direction_o,
    vector3_type const& normal
  ) const noexcept
  {
    return material_->PDFImportance(direction_i, direction_o, normal);
  }

  scatter_type
  SampleLight(
    vector3_type const& direction_i,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return material_->SampleLight(direction_i, normal, sampler);
  }

  scatter_type
  SampleImportance(
    vector3_type const& direction_i,
    vector3_type const& normal,
    Sampler* sampler
  ) const
  {
    return material_->SampleImportance(direction_i, normal, sampler);
  }

  std::vector<scatter_type>
  DistributionLight(
    vector3_type const& direction_i,
    vector3_type const& normal
  ) const
  {
    return material_->DistributionLight(direction_i, normal);
  }

  std::vector<scatter_type>
  DistributionImportance(
    vector3_type const& direction_i,
    vector3_type const& normal
  ) const
  {
    return material_->DistributionImportance(direction_i, normal);
  }

  radiant_type power() const noexcept {
    return Radiance() * SurfaceArea() * kPI;
  }
};

}
