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
#include "amber/prelude/kdtree.h"
#include "amber/prelude/sampling.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/algorithm.h"
#include "amber/rendering/algorithm_ups.h"
#include "amber/rendering/context.h"
#include "amber/rendering/kernel.h"
#include "amber/rendering/kernel_sequence.h"
#include "amber/rendering/multiple_importance_sampling.h"
#include "amber/rendering/parallel.h"
#include "amber/rendering/unified_path_sampling.h"

namespace amber {
namespace rendering {

template <typename Radiant>
class UnifiedPathSampling
: public Algorithm<Radiant>
{
public:
  UnifiedPathSampling(real_type initial_radius, real_type alpha) noexcept;

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
class UnifiedPathSampling<Radiant>::Thread
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
  struct Photon;
  using PhotonMap = KDTree<Photon>;

  const Scene<Radiant>& scene_;
  const Sensor& sensor_;
  KernelSequence& kernel_sequence_;
  MTSampler sampler_;
  std::vector<Subpath<Radiant>> light_paths_;
  std::vector<Photon> photons_;
  PhotonMap photon_map_;
  Subpath<Radiant> eye_path_;
  UnifiedPathSamplingBuffer<Radiant> ups_buffer_;
  SparseImage<Radiant> light_image_;

  void GenerateLightPaths();
  void GeneratePhotonMap();

  const Radiant Combine(
    const Subpath<Radiant>& light_path,
    const Subpath<Radiant>& eye_path,
    const Kernel& kernel,
    SparseImage<Radiant>& light_image
  );

  const Radiant ConnectEyeImage(
    const Subpath<Radiant>& light_path,
    const Subpath<Radiant>& eye_path,
    const Kernel& kernel
  );
  void ConnectLightImage(
    const Subpath<Radiant>& light_path,
    const Subpath<Radiant>& eye_path,
    const Kernel& kernel,
    SparseImage<Radiant>& light_image
  );
  const Radiant ConnectDensityEstimation(
    const Subpath<Radiant>& eye_path,
    const Kernel& kernel
  );

  const real_type MISWeight(
    const Subpath<Radiant>& light_path,
    const Subpath<Radiant>& eye_path,
    path_size_type s,
    path_size_type t,
    const Kernel& kernel,
    bool did_virtual_perturbation
  );
};

template <typename Radiant>
struct UnifiedPathSampling<Radiant>::Thread::Photon
{
  Vector3 position;
  pixel_size_type path_index;
  path_size_type event_index;

  Photon() noexcept;
  Photon(
    const Vector3& position,
    pixel_size_type path_index,
    path_size_type event_index
  ) noexcept;

  operator const Vector3&() const noexcept { return position; };
};



std::unique_ptr<Algorithm<RGB>>
MakeRGBUnifiedPathSampling(real_type initial_radius, real_type alpha)
{
  return std::make_unique<UnifiedPathSampling<RGB>>(initial_radius, alpha);
}

template <typename Radiant>
UnifiedPathSampling<Radiant>::UnifiedPathSampling(
  real_type initial_radius,
  real_type alpha
) noexcept
: initial_radius_(initial_radius)
, alpha_(alpha)
{}

template <typename Radiant>
const Image<Radiant>
UnifiedPathSampling<Radiant>::Render(
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
UnifiedPathSampling<Radiant>::Thread::Thread(
  const Scene<Radiant>& scene,
  const Sensor& sensor,
  KernelSequence& kernel_sequence
)
: scene_(scene)
, sensor_(sensor)
, kernel_sequence_(kernel_sequence)
, sampler_(std::random_device()())
, light_paths_(sensor_.Size())
, photons_()
, photon_map_()
, ups_buffer_()
, light_image_()
{}

template <typename Radiant>
UnifiedPathSampling<Radiant>::Thread::Thread(const Thread& thread)
: Thread(thread.scene_, thread.sensor_, thread.kernel_sequence_)
{}

template <typename Radiant>
const Image<Radiant>
UnifiedPathSampling<Radiant>::Thread::operator()()
{
  GenerateLightPaths();
  GeneratePhotonMap();
  const auto kernel = kernel_sequence_();

  auto image = sensor_.CreateImage<Radiant>();

  for (pixel_size_type y = 0; y < image.Height(); y++) {
    for (pixel_size_type x = 0; x < image.Width(); x++) {
      const auto pixel = Pixel(x, y);
      const auto& light_path = light_paths_[x + y * image.Width()];

      GenerateEyeSubpath(scene_, sensor_.Bind(pixel), sampler_, eye_path_);

      light_image_.Clear();
      image[pixel] += Combine(light_path, eye_path_, kernel, light_image_);
      image += light_image_;
    }
  }

  return image;
}

template <typename Radiant>
void
UnifiedPathSampling<Radiant>::Thread::GenerateLightPaths()
{
  for (pixel_size_type i = 0; i < sensor_.Size(); i++) {
    auto& light_path = light_paths_[i];
    GenerateLightSubpath(scene_, sampler_, light_path);
  }
}

template <typename Radiant>
void
UnifiedPathSampling<Radiant>::Thread::GeneratePhotonMap()
{
  photons_.clear();

  for (pixel_size_type i = 0; i < sensor_.Size(); i++) {
    const auto& light_path = light_paths_[i];
    for (path_size_type j = 0; j < light_path.Size(); j++) {
      const auto& event = light_path.begin()[j];
      if (scene_.Surface(event.Object()) == SurfaceType::Diffuse) {
        photons_.emplace_back(event.Position(), i, j);
      }
    }
  }

  photon_map_.Build(photons_);
}

template <typename Radiant>
const Radiant
UnifiedPathSampling<Radiant>::Thread::Combine(
  const Subpath<Radiant>& light_path,
  const Subpath<Radiant>& eye_path,
  const Kernel& kernel,
  SparseImage<Radiant>& light_image
)
{
  const auto contribution_eye = ConnectEyeImage(light_path, eye_path, kernel);
  ConnectLightImage(light_path, eye_path, kernel, light_image);
  const auto contribution_de = ConnectDensityEstimation(eye_path, kernel);

  return contribution_eye + contribution_de / sensor_.Size();
}

template <typename Radiant>
const Radiant
UnifiedPathSampling<Radiant>::Thread::ConnectEyeImage(
  const Subpath<Radiant>& light_path,
  const Subpath<Radiant>& eye_path,
  const Kernel& kernel
)
{
  Radiant measurement(0);

  for (path_size_type s = 0; s <= light_path.Size(); s++) {
    for (path_size_type t = 2; t <= eye_path.Size(); t++) {
      auto contribution = Connect(
        scene_,
        sensor_,
        s == 0 ? nullptr : &light_path.begin()[s - 1],
        t == 0 ? nullptr : &eye_path.begin()[t - 1]
      );

      if (!contribution) {
        continue;
      }

      measurement +=
        contribution.Value() *
        MISWeight(light_path, eye_path, s, t, kernel, true);
    }
  }

  return measurement;
}

template <typename Radiant>
void
UnifiedPathSampling<Radiant>::Thread::ConnectLightImage(
  const Subpath<Radiant>& light_path,
  const Subpath<Radiant>& eye_path,
  const Kernel& kernel,
  SparseImage<Radiant>& light_image
)
{
  for (path_size_type s = 1; s <= light_path.Size(); s++) {
    for (path_size_type t = 0; t < 2; t++) {
      auto contribution = Connect(
        scene_,
        sensor_,
        s == 0 ? nullptr : &light_path.begin()[s - 1],
        t == 0 ? nullptr : &eye_path.begin()[t - 1]
      );

      if (!contribution) {
        continue;
      }

      contribution *=
        MISWeight(light_path, eye_path, s, t, kernel, true) / sensor_.Size();

      light_image.Emplace(contribution);
    }
  }
}

template <typename Radiant>
const Radiant
UnifiedPathSampling<Radiant>::Thread::ConnectDensityEstimation(
  const Subpath<Radiant>& eye_path,
  const Kernel& kernel
)
{
  Radiant measurement(0);

  for (path_size_type t = 2; t <= eye_path.Size(); t++) {
    const auto& eye_end = eye_path.begin()[t - 1];
    if (scene_.Surface(eye_end.Object()) != SurfaceType::Diffuse) {
      continue;
    }

    photon_map_.Search(eye_end.Position(), kernel.radius(), 1024, photons_);

    Radiant contribution(0);

    for (const auto& photon : photons_) {
      const auto s = photon.event_index + 1;
      const auto& light_path = light_paths_[photon.path_index];
      const auto& light_end = light_path.begin()[photon.event_index];

      if (light_end.Object() != eye_end.Object()) {
        continue;
      }

      const auto bsdf = scene_.BSDF(
        eye_end.Object(),
        eye_end.Normal(),
        eye_end.DirectionOut(),
        light_end.DirectionOut()
      );
      const auto weight = MISWeight(light_path, eye_path, s, t, kernel, false);

      contribution += light_end.Weight() * bsdf * weight;
    }

    measurement += contribution * kernel() * eye_end.Weight();
  }

  return measurement;
}

template <typename Radiant>
const real_type
UnifiedPathSampling<Radiant>::Thread::MISWeight(
  const Subpath<Radiant>& light_path,
  const Subpath<Radiant>& eye_path,
  path_size_type s,
  path_size_type t,
  const Kernel& kernel,
  bool did_virtual_perturbation
)
{
  ups_buffer_.Buffer(
    scene_,
    sensor_,
    light_path.begin(),
    light_path.begin() + s,
    eye_path.begin(),
    eye_path.begin() + t,
    kernel(),
    did_virtual_perturbation
  );

  return ups_buffer_.MISWeight(PowerHeuristic<real_type>());
}

template <typename Radiant>
UnifiedPathSampling<Radiant>::Thread::Photon::Photon() noexcept
: position()
, path_index(std::numeric_limits<pixel_size_type>::max())
, event_index(std::numeric_limits<path_size_type>::max())
{}

template <typename Radiant>
UnifiedPathSampling<Radiant>::Thread::Photon::Photon(
  const Vector3& position,
  pixel_size_type path_index,
  path_size_type event_index
) noexcept
: position(position)
, path_index(path_index)
, event_index(event_index)
{}

}
}
