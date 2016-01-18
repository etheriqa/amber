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

#include <chrono>
#include <future>
#include <iomanip>
#include <iostream>
#include <locale>
#include <memory>
#include <sstream>
#include <thread>

#include "amber/cli/algorithm_factory.h"
#include "amber/cli/application.h"
#include "amber/cli/context.h"
#include "amber/cli/image.h"
#include "amber/cli/import.h"
#include "amber/cli/option.h"
#include "amber/etude/cornel_box.h"
#include "amber/postprocess/filmic.h"
#include "amber/postprocess/gamma.h"
#include "amber/raytracer/acceleration_bvh.h"
#include "amber/rendering/algorithm.h"
#include "amber/rendering/sensor.h"
#include "amber/scene/scene.h"

namespace amber {
namespace cli {

int
Application::Run(int argc, const char*const* argv)
{
  const auto option = ParseCommandLineOption(argc, argv);
  if (!option) {
    return -1;
  } else if (option->help) {
    return 0;
  }

  std::unique_ptr<rendering::Algorithm<RGB>> algorithm;
  try {
    algorithm = AlgorithmFactory()(*option);
  } catch (UnknownAlgorithmError const& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }

  std::unique_ptr<scene::Scene<RGB>> scene;
  if (option->scene.empty()) {
    scene = std::make_unique<scene::Scene<RGB>>(etude::CornelBox(
      0.050,
      0.050,
      6
    ));
  } else {
    auto data = ImportScene(option->scene);
    scene = std::make_unique<scene::Scene<RGB>>(
      scene::Scene<RGB>::Create<raytracer::BVH<real_type, scene::Object<RGB>>>(
        std::move(std::get<0>(data)),
        std::move(std::get<1>(data)),
        std::move(std::get<2>(data)),
        std::move(std::get<3>(data))
      )
    );
  }
  const auto sensor = rendering::Sensor(
    option->width,
    option->height,
    0.036,
    0.036 / option->width * option->height
  );

  Context context(option->n_threads, option->spp);
  if (option->time > 0) {
    std::thread([&](){
      std::this_thread::sleep_for(std::chrono::seconds(option->time));
      context.Expire();
    }).detach();
  }

  const auto image = Render(*option, *algorithm, *scene, sensor, context);

  {
    const auto gamma = amber::postprocess::Gamma();
    const auto filmic = amber::postprocess::Filmic();

    {
      const auto filename = option->output + ".png";
      std::cerr << "exporting " << filename << " (tonemapped) ... ";
      amber::cli::ExportPNG(gamma(filmic(image)), filename);
      std::cerr << "done." << std::endl;
    }

    {
      const auto filename = option->output + ".exr";
      std::cerr << "exporting " << filename << " (raw) ... ";
      amber::cli::ExportEXR(image, filename);
      std::cerr << "done." << std::endl;
    }
  }

  return 0;
}

const HDRImage
Application::Render(
  const CommandLineOption& option,
  rendering::Algorithm<RGB>& algorithm,
  const rendering::Scene<RGB>& scene,
  const rendering::Sensor& sensor,
  Context& context
)
{
  auto image = std::async(
    std::launch::async,
    [&](){ return algorithm.Render(scene, sensor, context); }
  );

  auto const initial = std::chrono::system_clock::now();

  for (;;) {
    auto const status = image.wait_for(std::chrono::milliseconds(500));

    if (context.IterationCount() == 0) {
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

    if (option.spp > 0) {
      ss << " - " << context.IterationCount() << "/" << option.spp;
    } else {
      ss << " - " << context.IterationCount() << " iterations";
    }

    ss << std::fixed << std::setprecision(2);
    if (option.spp > 0) {
      auto const percentage = 100. * context.IterationCount() / option.spp;
      ss << " - " << percentage << "%";
    }

    auto const ips = context.IterationCount() / (0.001 * ms);
    ss << " - " << ips << " iterations/second";

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
