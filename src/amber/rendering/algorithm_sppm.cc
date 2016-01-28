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

#include "amber/prelude/hit.h"
#include "amber/prelude/image.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/algorithm.h"
#include "amber/rendering/algorithm_sppm.h"
#include "amber/rendering/context.h"
#include "amber/rendering/kernel.h"
#include "amber/rendering/local_statistic.h"
#include "amber/rendering/parallel.h"
#include "amber/rendering/photon_map.h"
#include "amber/rendering/sensor.h"

namespace amber {
namespace rendering {

template <typename Radiant>
class StochasticPPM
: public Algorithm<Radiant>
{
public:
  StochasticPPM(real_type initial_radius, real_type alpha) noexcept;

  const Image<Radiant>
  Render(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    Context& context
  );

private:
  class Thread;
  using Kernel         = DiskKernel<real_type>;
  using LocalStatistic = LocalStatistic<real_type>;

  real_type initial_radius_;
  real_type alpha_;
};

template <typename Radiant>
class StochasticPPM<Radiant>::Thread
{
public:
  Thread(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    real_type alpha,
    std::vector<LocalStatistic>& statistics
  );
  Thread(const Thread& thread);

  const Image<Radiant> operator()();

private:
  using PhotonMap = PhotonMap<Radiant>;
  using Photon    = typename PhotonMap::Photon;

  const Scene<Radiant>& scene_;
  const Sensor& sensor_;
  real_type alpha_;
  std::vector<LocalStatistic>& statistics_;
  MTSampler sampler_;
  std::vector<Photon> photons_;
  PhotonMap photon_map_;

  void GeneratePhotonMap();
  const Radiant Render(const Sensor& sensor, const Kernel& kernel);
};



std::unique_ptr<Algorithm<RGB>>
MakeRGBStochasticPPM(real_type initial_radius, real_type alpha)
{
  return std::make_unique<StochasticPPM<RGB>>(initial_radius, alpha);
}

template <typename Radiant>
StochasticPPM<Radiant>::StochasticPPM(
  real_type initial_radius,
  real_type alpha
) noexcept
: initial_radius_(initial_radius)
, alpha_(alpha)
{}

template <typename Radiant>
const Image<Radiant>
StochasticPPM<Radiant>::Render(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Context& context
)
{
  std::vector<LocalStatistic> statistics;
  statistics.reserve(sensor.Size());
  for (pixel_size_type i = 0; i < sensor.Size(); i++) {
    statistics.emplace_back(initial_radius_);
  }

  return ParallelMean<Image<Radiant>>(
    context,
    sensor.CreateImage<Radiant>(),
    Thread(scene, sensor, alpha_, statistics)
  );
}

template <typename Radiant>
StochasticPPM<Radiant>::Thread::Thread(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  real_type alpha,
  std::vector<LocalStatistic>& statistic
)
: scene_(scene)
, sensor_(sensor)
, alpha_(alpha)
, statistics_(statistic)
, sampler_(std::random_device()())
, photons_()
, photon_map_()
{}

template <typename Radiant>
StochasticPPM<Radiant>::Thread::Thread(const Thread& thread)
: Thread(thread.scene_, thread.sensor_, thread.alpha_, thread.statistics_)
{}

template <typename Radiant>
const Image<Radiant>
StochasticPPM<Radiant>::Thread::operator()()
{
  GeneratePhotonMap();

  auto image = sensor_.CreateImage<Radiant>();

  for (pixel_size_type y = 0; y < image.Height(); y++) {
    for (pixel_size_type x = 0; x < image.Width(); x++) {
      auto& statistic = statistics_[x + y * image.Width()];
      std::lock_guard<LocalStatistic> lock(statistic);
      const auto kernel = statistic.Kernel();
      image[Pixel(x, y)] += Render(sensor_.Bind(Pixel(x, y)), kernel);
    }
  }

  return image;
}

template <typename Radiant>
void
StochasticPPM<Radiant>::Thread::GeneratePhotonMap()
{
  photon_map_.Build(scene_, sensor_.Size(), sampler_, photons_);
}

template <typename Radiant>
const Radiant
StochasticPPM<Radiant>::Thread::Render(
  const Sensor& sensor,
  const Kernel& kernel
)
{
  // TODO the following is the completely same as MPPM; possible to refactor
  auto generated = scene_.GenerateEyeRay(sensor, sampler_);
  auto& object  = std::get<ObjectPointer>(generated);
  auto& leading = std::get<Leading<Radiant>>(generated);

  auto ray    = leading.Ray();
  auto weight = leading.Weight();

  for (;;) {
    Hit hit;
    std::tie(object, hit) = scene_.Cast(ray);
    if (!hit) {
      break;
    }

    switch (scene_.Surface(object)) {
    case SurfaceType::Light:
      return weight * scene_.Radiance(object, hit.Normal(), -ray.Direction());
      break;
    case SurfaceType::Diffuse:
      {
        photon_map_.Search(hit.Position(), kernel.radius(), 1024, photons_);
        Radiant contribution(0);
        for (const auto& photon : photons_) {
          const auto bsdf = scene_.BSDF(
            object,
            hit.Normal(),
            -ray.Direction(),
            photon.DirectionOut()
          );
          contribution += photon.Weight() * bsdf;
        }
        return contribution * kernel() * weight;
      }
      break;
    default:
      break;
    }

    const auto scatter =
      scene_.SampleLight(object, hit.Normal(), -ray.Direction(), sampler_);
    const auto p_russian_roulette =
      std::min<real_type>(kRussianRoulette, Max(scatter.Weight()));

    if (prelude::Uniform<real_type>(sampler_) >= p_russian_roulette) {
      break;
    }

    ray = Ray(hit.Position(), scatter.DirectionIn());
    weight *= scatter.Weight() / p_russian_roulette;
  }

  return Radiant();
}

}
}
