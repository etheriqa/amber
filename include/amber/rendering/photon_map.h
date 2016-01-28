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

#pragma once

#include "amber/prelude/kdtree.h"
#include "amber/prelude/sampling.h"
#include "amber/rendering/forward.h"
#include "amber/rendering/leading.h"
#include "amber/rendering/object.h"
#include "amber/rendering/scatter.h"
#include "amber/rendering/scene.h"
#include "amber/rendering/surface.h"

namespace amber {
namespace rendering {

/** Photon map.
 */
template <typename Radiant>
class PhotonMap
{
public:
  class Photon;

  /** Constructor.
   */
  PhotonMap() noexcept;

  /** Builds a photon map.
   */
  void
  Build(
    const Scene<Radiant>& scene,
    std::size_t n_photons,
    Sampler& sampler,
    std::vector<Photon>& buffer
  );

  /** Query.
   */
  void
  Search(
    const Vector3& point,
    real_type radius,
    std::size_t k,
    std::vector<Photon>& buffer
  ) const;

private:
  KDTree<Photon> spatial_;
};

/** Photon.
 */
template <typename Radiant>
class PhotonMap<Radiant>::Photon
{
public:
  /** Constructors.
   */
  Photon() noexcept;
  Photon(
    const Vector3& position,
    const UnitVector3& direction_out,
    const Radiant& weight
  ) noexcept;

  /** Queries.
   */
  operator Vector3() const noexcept;
  const Vector3 Position() const noexcept;
  const UnitVector3 DirectionOut() const noexcept;
  const Radiant Weight() const noexcept;

private:
  Vector3 position_;
  UnitVector3 direction_out_;
  Radiant weight_;
};





template <typename Radiant>
PhotonMap<Radiant>::PhotonMap() noexcept
: spatial_()
{}

template <typename Radiant>
void
PhotonMap<Radiant>::Build(
  const Scene<Radiant>& scene,
  std::size_t n_photons,
  Sampler& sampler,
  std::vector<Photon>& buffer
)
{
  buffer.clear();

  for (std::size_t i = 0; i < n_photons; i++) {
    auto generated = scene.GenerateLightRay(sampler);
    auto& object  = std::get<ObjectPointer>(generated);
    auto& leading = std::get<Leading<Radiant>>(generated);

    auto ray    = leading.Ray();
    auto weight = leading.Weight();

    for (;;) {
      Hit hit;
      std::tie(object, hit) = scene.Cast(ray);
      if (!hit) {
        break;
      }

      if (scene.Surface(object) == SurfaceType::Diffuse) {
        buffer.emplace_back(hit.Position(), -ray.Direction(), weight / n_photons);
      }

      const auto scatter =
        scene.SampleImportance(object, hit.Normal(), -ray.Direction(), sampler);
      const auto p_russian_roulette =
        std::min<real_type>(kRussianRoulette, Max(scatter.Weight()));

      if (prelude::Uniform<real_type>(sampler) >= p_russian_roulette) {
        break;
      }

      ray = Ray(hit.Position(), scatter.DirectionIn());
      weight *= scatter.Weight() / p_russian_roulette;
    }
  }

  spatial_.Build(buffer);
}

template <typename Radiant>
void
PhotonMap<Radiant>::Search(
  const Vector3& point,
  real_type radius,
  std::size_t k,
  std::vector<Photon>& buffer
) const
{
  spatial_.Search(point, radius, k, buffer);
}

template <typename Radiant>
PhotonMap<Radiant>::Photon::Photon() noexcept
: position_()
, direction_out_()
, weight_(0)
{}

template <typename Radiant>
PhotonMap<Radiant>::Photon::Photon(
  const Vector3& position,
  const UnitVector3& direction_out,
  const Radiant& weight
) noexcept
: position_(position)
, direction_out_(direction_out)
, weight_(weight)
{}

template <typename Radiant>
PhotonMap<Radiant>::Photon::operator Vector3() const noexcept
{
  return position_;
}

template <typename Radiant>
const Vector3
PhotonMap<Radiant>::Photon::Position() const noexcept
{
  return position_;
}

template <typename Radiant>
const UnitVector3
PhotonMap<Radiant>::Photon::DirectionOut() const noexcept
{
  return direction_out_;
}

template <typename Radiant>
const Radiant
PhotonMap<Radiant>::Photon::Weight() const noexcept
{
  return weight_;
}

}
}
