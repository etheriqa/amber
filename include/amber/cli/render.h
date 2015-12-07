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
#include <iterator>
#include <locale>
#include <memory>
#include <sstream>
#include <thread>

#include "cli/option.h"
#include "core/acceleration.h"
#include "core/camera.h"
#include "core/component/light_sampler.h"
#include "core/shader.h"

namespace amber {
namespace cli {

template <
  typename InputIterator,
  typename Object = typename InputIterator::value_type
>
core::Image<typename Object::radiant_type>
render(
  InputIterator first,
  InputIterator last,
  std::shared_ptr<core::Shader<Object>> const& shader,
  std::shared_ptr<core::Acceleration<Object>> const& acceleration,
  core::Camera<typename Object::radiant_type, typename Object::real_type> const& camera,
  CommandLineOption const& option
)
{
  std::cerr
    << "Objects: " << std::distance(first, last) << std::endl
    << "Shader: " << *shader << std::endl
    << "Acceleration: " << *acceleration << std::endl
    << "Camera: " << camera << std::endl;

  core::Scene<Object> const scene(
    acceleration,
    std::make_shared<core::component::LightSampler<Object>>(first, last)
  );

  core::DefaultContext ctx(option.n_threads, option.spp);

  if (option.time > 0) {
    std::thread([&](){
      std::this_thread::sleep_for(std::chrono::seconds(option.time));
      ctx.Expire();
    }).detach();
  }

  auto image = std::async(
    std::launch::async,
    [&](){ return (*shader)(scene, camera, ctx); }
  );

  auto const initial = std::chrono::system_clock::now();
  for (;;) {
    auto const status = image.wait_for(std::chrono::milliseconds(100));

    if (ctx.IterationCount() == 0) {
      if (status == std::future_status::ready) {
        break;
      } else {
        continue;
      }
    }

    auto const elapsed = std::chrono::system_clock::now() - initial;
    auto const h =
      std::chrono::duration_cast<std::chrono::hours>(elapsed).count();
    auto const m =
      std::chrono::duration_cast<std::chrono::minutes>(elapsed).count();
    auto const s =
      std::chrono::duration_cast<std::chrono::seconds>(elapsed).count();
    auto const ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();

    auto const percentage = 100. * ctx.IterationCount() / option.spp;
    auto const ips = ctx.IterationCount() / (0.001 * ms);

    std::stringstream ss;
    ss.imbue(std::locale(""));
    ss << "\r";
    if (h) {
      ss << h << "h";
    }
    if (m) {
      ss << (m % 60) << "m";
    }
    ss << (s % 60) << "s";
    ss
      << " - "
      << ctx.IterationCount() << "/" << option.spp
      << " - "
      << std::fixed << std::setprecision(2)
      << percentage << "%"
      << " - "
      << ips << " iterations/second        ";
    std::cerr << ss.str() << std::flush;

    if (status == std::future_status::ready) {
      break;
    }
  }

  std::cerr << std::endl << std::endl;

  return image.get();
}

}
}
