#pragma once

#include "constant.h"
#include "material/lambertian.h"
#include "material/material.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Light : public Material<Radiant, RealType>
{
public:
  using material_type          = Material<Radiant, RealType>;

  using radiant_type           = typename material_type::radiant_type;
  using real_type              = typename material_type::real_type;
  using scattering_sample_type = typename material_type::ScatteringSample;
  using vector3_type           = typename material_type::vector3_type;

private:
  radiant_type m_emittance;

public:
  explicit Light(const radiant_type& e) : m_emittance(e) {}

  bool is_emissive() const noexcept
  {
    return true;
  }

  SurfaceType surface_type() const noexcept
  {
    return SurfaceType::diffuse;
  }

  radiant_type emittance() const noexcept
  {
    return m_emittance;
  }

  radiant_type bsdf(const vector3_type&, const vector3_type&, const vector3_type&) const noexcept
  {
    return radiant_type();
  }

  scattering_sample_type sample_scattering(const vector3_type& direction_i, const vector3_type& normal, Random& random) const
  {
    return Lambertian<radiant_type, real_type>(radiant_type()).sample_scattering(direction_i, normal, random);
  }
};

}
}
