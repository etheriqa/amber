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
#include "amber/prelude/ray.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/algorithm.h"
#include "amber/rendering/algorithm_lt.h"
#include "amber/rendering/context.h"
#include "amber/rendering/leading.h"
#include "amber/rendering/object.h"
#include "amber/rendering/parallel.h"
#include "amber/rendering/scatter.h"
#include "amber/rendering/scene.h"
#include "amber/rendering/sensor.h"
#include "amber/rendering/surface.h"

namespace amber {
namespace rendering {

template <typename Radiant>
class LightTracing
: public Algorithm<Radiant>
{
public:
  const Image<Radiant>
  Render(
    const Scene<Radiant>& scene,
    const Sensor& sensor,
    Context& context
  );

private:
  class Thread;
};

template <typename Radiant>
class LightTracing<Radiant>::Thread
{
public:
  Thread(const Scene<Radiant>& scene, const Sensor& sensor);
  Thread(const Thread&);

  const Image<Radiant> operator()();

private:
  const Scene<Radiant>& scene_;
  const Sensor& sensor_;
  MTSampler sampler_;

  void Render(Image<Radiant>& image);
};





std::unique_ptr<Algorithm<RGB>>
MakeRGBLightTracing()
{
  return std::make_unique<LightTracing<RGB>>();
}

template <typename Radiant>
const Image<Radiant>
LightTracing<Radiant>::Render(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  Context& context
)
{
  return ParallelMean<Image<Radiant>>(
    context,
    sensor.CreateImage<Radiant>(),
    Thread(scene, sensor)
  );
}

template <typename Radiant>
LightTracing<Radiant>::Thread::Thread(
  const Scene<Radiant>& scene,
  const Sensor& sensor
)
: scene_(scene)
, sensor_(sensor)
, sampler_(std::random_device()())
{}

template <typename Radiant>
LightTracing<Radiant>::Thread::Thread(const Thread& thread)
: Thread(thread.scene_, thread.sensor_)
{}

template <typename Radiant>
Image<Radiant> const
LightTracing<Radiant>::Thread::operator()()
{
  auto image = sensor_.CreateImage<Radiant>();
  for (pixel_size_type i = 0; i < image.Size(); i++) {
    Render(image);
  }
  return image;
}

template <typename Radiant>
void
LightTracing<Radiant>::Thread::Render(Image<Radiant>& image)
{
  auto generated = scene_.GenerateLightRay(sampler_);
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

    if (scene_.Surface(object) == SurfaceType::Eye) {
      const auto response =
        scene_.Response(sensor_, hit.position, -ray.direction);
      if (response.Pixel()) {
        image[response.Pixel()] += weight * response.Value() / image.Size();
      }
    }

    const auto scatter =
      scene_.SampleImportance(object, hit.normal, -ray.direction, sampler_);

    const auto p_russian_roulette =
      std::min<real_type>(kRussianRoulette, Max(scatter.Weight()));

    if (prelude::Uniform<real_type>(sampler_) >= p_russian_roulette) {
      break;
    }

    ray = Ray(hit.position, scatter.DirectionIn());
    weight *= scatter.Weight() / p_russian_roulette;
  }
}

}
}
