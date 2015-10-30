/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <memory>

#include "base/writer.h"
#include "camera/camera.h"
#include "cli/shading_progress.h"

namespace amber {
namespace shader {

template <typename Acceleration>
struct Shader : public Writer {
  using shader_type              = Shader<Acceleration>;
  using acceleration_type        = Acceleration;

  using object_buffer_type       = std::vector<typename acceleration_type::object_type>;
  using object_type              = typename acceleration_type::object_type;

  using hit_type                 = typename object_type::hit_type;
  using radiant_type             = typename object_type::radiant_type;
  using radiant_value_type       = typename object_type::radiant_value_type;
  using ray_type                 = typename object_type::ray_type;
  using real_type                = typename object_type::real_type;

  using camera_type              = camera::Camera<radiant_type, real_type>;
  using progress_const_reference = std::shared_ptr<cli::ShadingProgress const>;
  using progress_reference       = std::shared_ptr<cli::ShadingProgress>;
  using progress_type            = cli::ShadingProgress;

  virtual progress_const_reference render(const object_buffer_type&, const camera_type&) const = 0;
};

}
}
