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
#include "amber/rendering/algorithm_bdpt.h"
#include "amber/rendering/bidirectional_path_sampling.h"
#include "amber/rendering/context.h"
#include "amber/rendering/multiple_importance_sampling.h"
#include "amber/rendering/parallel.h"

namespace amber {
namespace rendering {

template <typename Radiant>
class BidirectionalPathTracing
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
class BidirectionalPathTracing<Radiant>::Thread
{
public:
  Thread(const Scene<Radiant>& scene, const Sensor& sensor);
  Thread(const Thread& thread);

  const Image<Radiant> operator()();

private:
  const Scene<Radiant>& scene_;
  const Sensor& sensor_;
  MTSampler sampler_;
  Subpath<Radiant> light_path_;
  Subpath<Radiant> eye_path_;
  BidirectionalPathSamplingBuffer<Radiant> bdps_buffer_;
  SparseImage<Radiant> light_image_;

  const Radiant Render(const Sensor& sensor, SparseImage<Radiant>& light_image);
};



std::unique_ptr<Algorithm<RGB>>
MakeRGBBidirectionalPathTracing()
{
  return std::make_unique<BidirectionalPathTracing<RGB>>();
}

template <typename Radiant>
const Image<Radiant>
BidirectionalPathTracing<Radiant>::Render(
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
BidirectionalPathTracing<Radiant>::Thread::Thread(
  const Scene<Radiant>& scene,
  const Sensor& sensor
)
: scene_(scene)
, sensor_(sensor)
, sampler_(std::random_device()())
, light_path_()
, eye_path_()
, bdps_buffer_()
, light_image_()
{}

template <typename Radiant>
BidirectionalPathTracing<Radiant>::Thread::Thread(const Thread& thread)
: Thread(thread.scene_, thread.sensor_)
{}

template <typename Radiant>
const Image<Radiant>
BidirectionalPathTracing<Radiant>::Thread::operator()()
{
  auto image = sensor_.CreateImage<Radiant>();
  for (pixel_size_type y = 0; y < image.Height(); y++) {
    for (pixel_size_type x = 0; x < image.Width(); x++) {
      light_image_.Clear();
      image[Pixel(x, y)] += Render(sensor_.Bind(Pixel(x, y)), light_image_);
      image += light_image_;
    }
  }
  return image;
}

template <typename Radiant>
const Radiant
BidirectionalPathTracing<Radiant>::Thread::Render(
  const Sensor& sensor,
  SparseImage<Radiant>& light_image
)
{
  GenerateLightSubpath(scene_, sampler_, light_path_);
  GenerateEyeSubpath(scene_, sensor, sampler_, eye_path_);

  return Combine(
    scene_,
    sensor,
    light_path_,
    eye_path_,
    BalanceHeuristic(),
    bdps_buffer_,
    light_image
  );
}

}
}
