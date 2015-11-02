/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <list>
#include <locale>
#include <sstream>
#include <thread>
#include "camera/camera.h"
#include "shader/shader.h"

namespace amber {
namespace cli {

template <typename Scene,
          typename Shader = typename shader::Shader<Scene>,
          typename Object = typename Shader::object_type,
          typename Camera = typename Shader::camera_type,
          typename Image = typename Shader::image_type>
Image render(Shader* shader,
             std::vector<Object> const& objects,
             Camera const& camera) {
  {
    std::vector<Object> dummy_objects;
    std::cerr
      << "Shader: " << *shader << std::endl
      //<< "Acceleration: " << Acceleration(dummy_objects.begin(), dummy_objects.end()) << std::endl
      << "Objects: " << objects.size() << std::endl
      << "Camera: " << camera << std::endl
      << std::endl;
  }

  Scene const scene(objects.begin(), objects.end());

  auto image =
    std::async(std::launch::async, [&](){ return (*shader)(scene, camera); });

  size_t phase = 0;
  while (image.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
    auto const& progress = shader->progress();
    if (progress.current_phase.load() > phase) {
      std::cerr << std::endl;
    }
    phase = progress.current_phase.load();
    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss
      << "\r"
      << "[" << progress.current_phase
      << "/" << progress.total_phase
      << "] " << progress.phase;
    if (progress.total_job > 0) {
      ss
        << " " << progress.current_job
        << "/" << progress.total_job
        << " " << std::fixed << std::setprecision(2)
        << 100. * progress.current_job / progress.total_job << "%";
    }
    std::cerr << ss.str() << std::flush;
  }

  std::cerr << std::endl;

  return image.get();
}

}
}
