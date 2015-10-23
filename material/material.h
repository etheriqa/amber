#pragma once

#include <numeric>
#include <vector>
#include "material/surface_type.h"
#include "random.h"

namespace amber {
namespace material {

template <typename Radiant, typename RealType>
class Material {
public:
  using material_type = Material<Radiant, RealType>;
  using radiant_type  = Radiant;
  using real_type     = RealType;
  using vector3_type  = geometry::Vector3<RealType>;

  struct ScatteringSample
  {
    vector3_type direction_o;
    radiant_type bsdf;
    real_type psa_probability;
  };

  virtual SurfaceType surfaceType() const noexcept = 0;
  virtual bool isEmissive() const noexcept { return false; }
  virtual radiant_type emittance() const noexcept { return radiant_type(); }

  virtual radiant_type
  bsdf(const vector3_type& direction_i,
       const vector3_type& direction_o,
       const vector3_type& normal) const noexcept { return radiant_type(); }

  virtual ScatteringSample
  sampleScattering(const radiant_type& radiant,
                   const vector3_type& direction_i,
                   const vector3_type& normal,
                   Random& random) const {
    const auto candidates = scatteringCandidates(radiant, direction_i, normal);
    if (candidates.size() == 1) {
      return candidates.front();
    }

    const auto r = random.uniform(std::accumulate(
      candidates.begin(), candidates.end(), static_cast<real_type>(0),
      [](const auto& acc, const auto& s){ return acc + s.psa_probability; }));
    real_type p = 0;
    for (const auto& s : candidates) {
      p += s.psa_probability;
      if (r < p) {
        return s;
      }
    }
    return candidates.back();
  }

  virtual std::vector<ScatteringSample>
  scatteringCandidates(const radiant_type& radiant,
                       const vector3_type& direction_i,
                       const vector3_type& normal) const {
    throw std::logic_error("scatteringCandidates is not implemented");
  }
};

}
}
