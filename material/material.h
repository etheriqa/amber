#pragma once

#include "material/surface_type.h"
#include "random.h"
#include "ray.h"
#include "vector.h"

namespace amber {
namespace material {

template <typename Radiant>
struct Material
{
  using material_type = Material<Radiant>;
  using radiant_type  = Radiant;

  using real_type     = typename radiant_type::real_type;

  using vector3_type  = Vector3<real_type>;

  struct ScatteringSample
  {
    vector3_type direction_o;
    radiant_type bsdf;
    real_type psa_probability;
  };

  virtual ~Material() {}

  virtual bool is_emissive() const noexcept = 0;
  virtual SurfaceType surface_type() const noexcept = 0;
  virtual radiant_type emittance() const noexcept = 0;
  virtual radiant_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept = 0;
  virtual ScatteringSample sample_scattering(const vector3_type&, const vector3_type&, Random&) const = 0;
};

}
}
