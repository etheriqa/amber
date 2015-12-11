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

#include "core/material.h"
#include "core/primitive.h"

namespace amber {
namespace core {

template <typename Radiant, typename RealType>
class Object
{
public:
  using primitive_type = Primitive<RealType>;
  using material_type  = Material<Radiant, RealType>;

  using real_type    = RealType; // TODO remove
  using radiant_type = Radiant;  // TODO remove

  using aabb_type = typename primitive_type::aabb_type;
  using hit_type  = typename primitive_type::hit_type;
  using ray_type  = typename primitive_type::ray_type;

  using radiant_value_type = typename material_type::radiant_value_type;
  using scatter_type       = typename material_type::scatter_type;
  using unit_vector3_type  = typename material_type::unit_vector3_type;

  struct Hash
  {
    std::size_t operator()(Object const& o) const noexcept {
      return
        std::hash<std::size_t>()(reinterpret_cast<std::size_t>(o.primitive_)) +
        std::hash<std::size_t>()(reinterpret_cast<std::size_t>(o.material_));
    }
  };

  struct EqualTo
  {
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

  operator bool() const noexcept
  {
    return primitive_ != nullptr && material_ != nullptr;
  }

  bool operator==(Object const& o) const noexcept
  {
    return primitive_ == o.primitive_ && material_ == o.material_;
  }

  bool operator!=(Object const& o) const noexcept
  {
    return !(*this == o);
  }

  RealType SurfaceArea() const noexcept
  {
    return primitive_->SurfaceArea();
  }

  aabb_type BoundingBox() const noexcept
  {
    return primitive_->BoundingBox();
  }

  hit_type Intersect(ray_type const& ray) const noexcept
  {
    return primitive_->Intersect(ray);
  }

  ray_type SamplePoint(Sampler& sampler) const
  {
    return primitive_->SamplePoint(sampler);
  }

  SurfaceType Surface() const noexcept
  {
    return material_->Surface();
  }

  Radiant
  Irradiance() const noexcept
  {
    return material_->Irradiance();
  }

  Radiant
  Radiance(
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    return material_->Radiance(direction_o, normal);
  }

  Radiant
  BSDF(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    return material_->BSDF(direction_i, direction_o, normal);
  }

  Radiant
  AdjointBSDF(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    return material_->AdjointBSDF(direction_i, direction_o, normal);
  }

  radiant_value_type
  PDFLight(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    return material_->PDFLight(direction_i, direction_o, normal);
  }

  radiant_value_type
  PDFImportance(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& direction_o,
    unit_vector3_type const& normal
  ) const noexcept
  {
    return material_->PDFImportance(direction_i, direction_o, normal);
  }

  scatter_type
  SampleLight(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& normal,
    Sampler& sampler
  ) const
  {
    return material_->SampleLight(direction_i, normal, sampler);
  }

  scatter_type
  SampleImportance(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& normal,
    Sampler& sampler
  ) const
  {
    return material_->SampleImportance(direction_i, normal, sampler);
  }

  std::vector<scatter_type>
  DistributionLight(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& normal
  ) const
  {
    return material_->DistributionLight(direction_i, normal);
  }

  std::vector<scatter_type>
  DistributionImportance(
    unit_vector3_type const& direction_i,
    unit_vector3_type const& normal
  ) const
  {
    return material_->DistributionImportance(direction_i, normal);
  }

  Radiant Power() const noexcept
  {
    return Irradiance() * SurfaceArea();
  }
};

}
}
