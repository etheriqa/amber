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
#include "amber/rendering/algorithm_mppm.h"
#include "amber/rendering/context.h"
#include "amber/rendering/kernel.h"
#include "amber/rendering/kernel_sequence.h"
#include "amber/rendering/parallel.h"
#include "amber/rendering/photon_map.h"
#include "amber/rendering/sensor.h"

namespace amber {
namespace rendering {

template <typename Radiant>
class MemorylessPPM
: public Algorithm<Radiant>
{
public:
  MemorylessPPM(real_type initial_radius, real_type alpha) noexcept;

  const Image<Radiant>
  Render(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    Context& context
  );

private:
  class Thread;
  using Kernel         = DiskKernel<real_type>;
  using KernelSequence = KernelSequence<real_type, Kernel>;

  real_type initial_radius_;
  real_type alpha_;
};

template <typename Radiant>
class MemorylessPPM<Radiant>::Thread
{
public:
  Thread(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    KernelSequence& kernel_sequence
  );
  Thread(const Thread& thread);

  const Image<Radiant> operator()();

private:
  using PhotonMap = PhotonMap<Radiant>;
  using Photon    = typename PhotonMap::Photon;

  const Scene<Radiant>& scene_;
  const Sensor& sensor_;
  KernelSequence& kernel_sequence_;
  MTSampler sampler_;
  std::vector<Photon> photons_;
  PhotonMap photon_map_;

  void GeneratePhotonMap();
  const Radiant Render(const Sensor& sensor, const Kernel& kernel);
};



std::unique_ptr<Algorithm<RGB>>
MakeRGBMemorylessPPM(real_type initial_radius, real_type alpha)
{
  return std::make_unique<MemorylessPPM<RGB>>(initial_radius, alpha);
}

template <typename Radiant>
MemorylessPPM<Radiant>::MemorylessPPM(
  real_type initial_radius,
  real_type alpha
) noexcept
: initial_radius_(initial_radius)
, alpha_(alpha)
{}

template <typename Radiant>
const Image<Radiant>
MemorylessPPM<Radiant>::Render(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Context& context
)
{
  KernelSequence kernel_sequence(initial_radius_, alpha_);

  return ParallelMean<Image<Radiant>>(
    context,
    sensor.CreateImage<Radiant>(),
    Thread(scene, sensor, kernel_sequence)
  );
}

template <typename Radiant>
MemorylessPPM<Radiant>::Thread::Thread(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  KernelSequence& kernel_sequence
)
: scene_(scene)
, sensor_(sensor)
, kernel_sequence_(kernel_sequence)
, sampler_(std::random_device()())
, photons_()
, photon_map_()
{}

template <typename Radiant>
MemorylessPPM<Radiant>::Thread::Thread(const Thread& thread)
: Thread(thread.scene_, thread.sensor_, thread.kernel_sequence_)
{}

template <typename Radiant>
const Image<Radiant>
MemorylessPPM<Radiant>::Thread::operator()()
{
  GeneratePhotonMap();
  const auto kernel = kernel_sequence_();

  auto image = sensor_.CreateImage<Radiant>();

  for (pixel_size_type y = 0; y < image.Height(); y++) {
    for (pixel_size_type x = 0; x < image.Width(); x++) {
      image[Pixel(x, y)] += Render(sensor_.Bind(Pixel(x, y)), kernel);
    }
  }

  return image;
}

template <typename Radiant>
void
MemorylessPPM<Radiant>::Thread::GeneratePhotonMap()
{
  photon_map_.Build(scene_, sensor_.Size(), sampler_, photons_);
}

template <typename Radiant>
const Radiant
MemorylessPPM<Radiant>::Thread::Render(
  const Sensor& sensor,
  const Kernel& kernel
)
{
  // TODO the following is the completely same as SPPM; possible to refactor
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
      return weight * scene_.Radiance(object, hit.normal, -ray.direction);
      break;
    case SurfaceType::Diffuse:
      {
        photon_map_.Search(hit.position, kernel.radius(), 1024, photons_);
        Radiant contribution(0);
        for (const auto& photon : photons_) {
          const auto bsdf = scene_.BSDF(
            object,
            hit.normal,
            -ray.direction,
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
      scene_.SampleLight(object, hit.normal, -ray.direction, sampler_);
    const auto p_russian_roulette =
      std::min<real_type>(kRussianRoulette, Max(scatter.Weight()));

    if (prelude::Uniform<real_type>(sampler_) >= p_russian_roulette) {
      break;
    }

    ray = Ray(hit.position, scatter.DirectionIn());
    weight *= scatter.Weight() / p_russian_roulette;
  }

  return Radiant();
}

}
}
