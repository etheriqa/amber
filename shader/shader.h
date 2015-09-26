#pragma once

#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "shader/progress.h"

namespace amber {
namespace shader {

template <class Scene>
struct Shader
{
  using shader_type              = Shader<Scene>;
  using scene_type               = Scene;

  using flux_type                = typename scene_type::flux_type;
  using hit_type                 = typename scene_type::hit_type;
  using object_type              = typename scene_type::object_type;
  using ray_type                 = typename scene_type::ray_type;
  using real_type                = typename scene_type::real_type;

  using camera_type              = Camera<flux_type>;
  using progress_const_reference = std::shared_ptr<const ShadingProgress>;
  using progress_reference       = std::shared_ptr<ShadingProgress>;
  using progress_type            = ShadingProgress;

  virtual std::string to_string() const = 0;
  virtual progress_const_reference render(const scene_type&, const camera_type&) const = 0;
};

}
}
