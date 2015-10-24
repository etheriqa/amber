#pragma once

#include <functional>
#include "material/surface_type.h"

namespace amber {
namespace object {

template <typename Primitive, typename Material>
class Object {
public:
  using primitive_type          = Primitive;
  using material_type           = Material;

  using aabb_type               = typename primitive_type::aabb_type;
  using hit_type                = typename primitive_type::hit_type;
  using initial_ray_sample_type = typename primitive_type::initial_ray_sample_type;
  using ray_type                = typename primitive_type::ray_type;
  using real_type               = typename primitive_type::real_type;
  using radiant_type            = typename material_type::radiant_type;
  using radiant_value_type      = typename material_type::radiant_value_type;
  using scattering_sample_type  = typename material_type::ScatteringSample;
  using vector3_type            = typename material_type::vector3_type;

  using primitive_reference     = primitive_type*;
  using material_reference      = material_type*;

  struct Hash {
    size_t operator()(const Object& o) const noexcept {
      return
        std::hash<size_t>()(reinterpret_cast<size_t>(o.primitive_)) +
        std::hash<size_t>()(reinterpret_cast<size_t>(o.material_));
    }
  };

  struct EqualTo {
    bool operator()(const Object& a, const Object& b) const noexcept {
      return a == b;
    }
  };

private:
  primitive_reference primitive_;
  material_reference material_;

public:
  Object() noexcept : primitive_(nullptr), material_(nullptr) {}

  Object(const primitive_reference& primitive,
         const material_reference& material)
    : primitive_(primitive), material_(material) {}

  bool operator==(const Object& o) const noexcept {
    return primitive_ == o.primitive_ && material_ == o.material_;
  }

  bool operator!=(const Object& o) const noexcept {
    return !(*this == o);
  }

  real_type surface_area() const noexcept {
    return primitive_->surface_area();
  }

  aabb_type aabb() const noexcept {
    return primitive_->aabb();
  }

  hit_type intersect(const ray_type& ray) const noexcept {
    return primitive_->intersect(ray);
  }

  initial_ray_sample_type sample_initial_ray(Random& random) const {
    return primitive_->sample_initial_ray(random);
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

  radiant_type bsdf(const vector3_type& direction_i,
                    const vector3_type& direction_o,
                    const vector3_type& normal) const noexcept {
    return material_->bsdf(direction_i, direction_o, normal);
  }

  scattering_sample_type sampleScattering(const radiant_type& radiant,
                                          const vector3_type& direction_i,
                                          const vector3_type& normal,
                                          Random& random) const {
    return material_->sampleScattering(radiant, direction_i, normal, random);
  }

  std::vector<scattering_sample_type>
  scatteringCandidates(const radiant_type& radiant,
                       const vector3_type& direction_i,
                       const vector3_type& normal) const {
    return material_->scatteringCandidates(radiant, direction_i, normal);
  }

  radiant_type power() const noexcept {
    return emittance() * surface_area() * kPI;
  }
};

}
}
