#pragma once

#include "random.h"
#include "ray.h"
#include "vector.h"

namespace amber {
namespace material {

template <typename Flux>
struct Material
{
  using material_type   = Material<Flux>;
  using flux_type       = Flux;

  using real_type       = typename flux_type::real_type;

  using vector3_type    = Vector3<real_type>;

  struct ScatteringSample
  {
    vector3_type direction_o;
    flux_type bsdf;
    real_type psa_probability;
  };

  virtual ~Material() {}

  virtual bool is_emissive() const noexcept = 0;
  virtual bool is_specular() const noexcept = 0;
  virtual flux_type emittance() const noexcept = 0;
  virtual flux_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept = 0;
  virtual ScatteringSample sample_scattering(const vector3_type&, const vector3_type&, Random&) const = 0;
};

}
}
