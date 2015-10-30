/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <chrono>
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

template <typename Acceleration,
          typename Object = typename Acceleration::object_type>
void render(
  shader::Shader<Acceleration>* shader,
  std::vector<Object> const& objects,
  const typename shader::Shader<Acceleration>::camera_type& camera
)
{
  {
    std::vector<Object> dummy_objects;
    std::cerr
      << "Shader: " << *shader << std::endl
      << "Acceleration: " << Acceleration(dummy_objects.begin(), dummy_objects.end()) << std::endl
      << "Objects: " << objects.size() << std::endl
      << "Camera: " << camera << std::endl
      << std::endl;
  }

  const auto progress = shader->render(objects, camera);
  std::list<size_t> n_done_history({0});

  do {
    using namespace std::literals;
    std::this_thread::sleep_for(1s);

    n_done_history.push_back(progress->n_done());
    while (n_done_history.size() > 10) {
      n_done_history.pop_front();
    }

    const auto elapsed = std::chrono::system_clock::now() - progress->begin_time();
    const auto estimated = elapsed + 1s * n_done_history.size() * (progress->n_task() - progress->n_done()) / std::max<size_t>(1, progress->n_done() - n_done_history.front());

    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss << std::fixed << std::setprecision(2)
      << "\r"
      << std::setw(2) << std::setfill('0')
      << std::chrono::duration_cast<std::chrono::hours>(elapsed).count()
      << ":"
      << std::setw(2) << std::setfill('0')
      << std::chrono::duration_cast<std::chrono::minutes>(elapsed).count() % 60
      << ":"
      << std::setw(2) << std::setfill('0')
      << std::chrono::duration_cast<std::chrono::seconds>(elapsed).count() % 60
      << "/"
      << std::setw(2) << std::setfill('0')
      << std::chrono::duration_cast<std::chrono::hours>(estimated).count()
      << ":"
      << std::setw(2) << std::setfill('0')
      << std::chrono::duration_cast<std::chrono::minutes>(estimated).count() % 60
      << ":"
      << std::setw(2) << std::setfill('0')
      << std::chrono::duration_cast<std::chrono::seconds>(estimated).count() % 60
      << "\t"
      << progress->n_done() << "/" << progress->n_task()
      << "\t"
      << std::setprecision(2)
      << 100.0 * progress->n_done() / progress->n_task() << "%";
    std::cerr << ss.str() << std::flush;
  } while (!progress->is_done());

  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(progress->duration());

  std::cerr << std::endl << std::endl;

  {
    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss
      << std::setprecision(3)
      << duration.count() / 1000.0 << "s" << std::endl
      << progress->n_task() * 1000 / duration.count() << " samples/sec" << std::endl;
    std::cerr << ss.str() << std::flush;
  }
}

}
}
