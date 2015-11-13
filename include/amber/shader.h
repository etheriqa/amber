/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include "camera.h"
#include "image.h"
#include "progress.h"
#include "writer.h"

namespace amber {

template <typename Scene>
class Shader : public Writer {
public:
  using scene_type  = Scene;

  using camera_type = Camera<typename Scene::object_type::radiant_type,
                             typename Scene::object_type::real_type>;
  using image_type  = Image<typename Scene::object_type::radiant_type>;
  using object_type = typename Scene::object_type;

  virtual Progress const& progress() const noexcept = 0;
  virtual image_type operator()(Scene const&, camera_type const&) = 0;
};

}