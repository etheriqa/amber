/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <functional>
#include "material/surface_type.h"

namespace amber {
namespace object {

template <typename Primitive, typename Material>
class Object {
public:
  using primitive_type     = Primitive;
  using material_type      = Material;

  using aabb_type          = typename primitive_type::aabb_type;
  using first_ray_type     = typename primitive_type::first_ray_type;
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
  primitive_type* primitive_;
  material_type* material_;

public:
  Object() noexcept : primitive_(nullptr), material_(nullptr) {}

  Object(primitive_type* primitive, material_type* material) noexcept
    : primitive_(primitive), material_(material) {}

  bool operator==(Object const& o) const noexcept {
    return primitive_ == o.primitive_ && material_ == o.material_;
  }

  bool operator!=(Object const& o) const noexcept {
    return !(*this == o);
  }

  real_type surfaceArea() const noexcept {
    return primitive_->surfaceArea();
  }

  aabb_type aabb() const noexcept {
    return primitive_->aabb();
  }

  hit_type intersect(ray_type const& ray) const noexcept {
    return primitive_->intersect(ray);
  }

  first_ray_type sampleFirstRay(Sampler* sampler) const {
    return primitive_->sampleFirstRay(sampler);
  }

  material::SurfaceType surfaceType() const noexcept {
    return material_->surfaceType();
  }

  bool isEmissive() const noexcept {
    return material_->isEmissive();
  }

  radiant_type emittance() const noexcept {
    return material_->emittance();
  }

  radiant_type bsdf(vector3_type const& direction_i,
                    vector3_type const& direction_o,
                    vector3_type const& normal) const noexcept {
    return material_->bsdf(direction_i, direction_o, normal);
  }

  scatter_type sampleScatter(radiant_type const& radiant,
                                          vector3_type const& direction_i,
                                          vector3_type const& normal,
                                          Sampler* sampler) const {
    return material_->sampleScatter(radiant, direction_i, normal, sampler);
  }

  std::vector<scatter_type>
  specularScatters(radiant_type const& radiant,
                       vector3_type const& direction_i,
                       vector3_type const& normal) const {
    return material_->specularScatters(radiant, direction_i, normal);
  }

  radiant_type power() const noexcept {
    return emittance() * surfaceArea() * kPI;
  }
};

}
}
