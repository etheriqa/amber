/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "base/writer.h"
#include "camera/camera.h"
#include "camera/image.h"
#include "shader/progress.h"

namespace amber {
namespace shader {

template <typename Scene>
class Shader : public Writer {
public:
  using scene_type  = Scene;

  using camera_type = camera::Camera<typename Scene::object_type::radiant_type,
                                     typename Scene::object_type::real_type>;
  using image_type  = camera::Image<typename Scene::object_type::radiant_type>;
  using object_type = typename Scene::object_type;

  virtual Progress const& progress() const noexcept = 0;
  virtual image_type operator()(Scene const&, camera_type const&) = 0;
};

}
}
