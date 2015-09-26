#pragma once

#include "hit.h"
#include "random.h"
#include "ray.h"
#include "vector.h"

namespace amber {
namespace material {

template <class Flux>
struct Material
{
  using material_type   = Material<Flux>;
  using flux_type       = Flux;

  using real_type       = typename flux_type::real_type;

  using hit_type        = Hit<real_type>;
  using ray_sample_type = RaySample<flux_type>;
  using ray_type        = Ray<real_type>;
  using vector3_type    = Vector3<real_type>;

  virtual ~Material() {}

  virtual bool is_emissive() const noexcept = 0;
  virtual bool is_specular() const noexcept = 0;
  virtual flux_type emittance() const noexcept = 0;
  virtual flux_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept = 0;
  virtual ray_sample_type sample_ray_bsdf(const hit_type&, const ray_type&, Random&) const = 0;
};

}
}
