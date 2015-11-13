// Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
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

#pragma once

#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <list>
#include <locale>
#include <sstream>
#include <thread>

#include "camera.h"
#include "shader.h"

namespace amber {
namespace cli {

template <typename Scene,
          typename Shader = typename amber::Shader<Scene>,
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
