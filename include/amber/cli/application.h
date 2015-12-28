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

#include <iostream>
#include <string>

#include "cli/acceleration_factory.h"
#include "cli/export.h"
#include "cli/import.h"
#include "cli/option.h"
#include "cli/render.h"
#include "cli/shader_factory.h"
#include "core/camera.h"
#include "core/lens/thin.h"
#include "core/material/eye.h"
#include "core/object.h"
#include "core/primitive/regular_polygon.h"
#include "core/rgb.h"
#include "core/scene.h"
#include "core/vector3.h"
#include "scene/cornel_box.h"
#include "scene/cornel_box_complex.h"

namespace amber {
namespace cli {

class Application {
private:
  using size_type          = std::size_t;
  using real_type          = std::double_t;
  using vector3_type       = core::Vector3<real_type>;

  using radiant_value_type = std::float_t;
  using radiant_type       = core::RGB<radiant_value_type>;

  using object_type        = core::Object<radiant_type, real_type>;

  using acceleration_type  = core::acceleration::KDTree<object_type>;

  using scene_type         = core::Scene<acceleration_type>;

  int argc_;
  char **argv_;

public:
  Application(int argc, char **argv) noexcept : argc_(argc), argv_(argv) {}

  int run()
  {
    auto const option = ParseCommandLineOption(argc_, argv_);
    if (!option) {
      return -1;
    } else if (option->help) {
      return 0;
    }

    auto objects = scene::CornelBox();

    vector3_type const origin(0, 0, 4);
    vector3_type const axis(0, 0, -1);
    vector3_type const up(0, 1, 0);
    size_type const n_blades = 6;
    real_type const radius = 0.05;
    real_type const focus_distance = 4;

    auto const aperture = core::primitive::RegularPolygon<real_type>(
      origin, axis, up, n_blades, radius
    );
    auto const lens = core::lens::Thin<real_type>(focus_distance);
    auto const sensor = core::Sensor<real_type>(
      option->width * option->ssaa,
      option->height * option->ssaa
    );
    auto const camera = core::Camera<radiant_type, real_type>(
      sensor, &lens, &aperture, origin, axis, up
    );
    objects.push_back(camera.aperture());

    ShaderFactory<object_type>::shader_ptr shader;
    try {
      shader = ShaderFactory<object_type>()(*option);
    } catch (UnknownShaderError const& e) {
      std::cerr << e.what() << std::endl;
      return -1;
    }

    AccelerationFactory<object_type>::acceleration_ptr acceleration;
    try {
      acceleration = AccelerationFactory<object_type>()(
        objects.begin(),
        objects.end(),
        *option
      );
    } catch (UnknownAccelerationError const& e) {
      std::cerr << e.what() << std::endl;
      return -1;
    }

    auto const image = render(
      objects.begin(),
      objects.end(),
      shader,
      acceleration,
      camera,
      *option
    );

    if (option->reference != "") {
      auto const reference = ImportEXR(option->reference);
      std::cerr
        << "Mean Square Error = "
        << MeanSquareError(image, reference)
        << std::endl
        << "Average Relative Error = "
        << AverageRelativeError(image, reference)
        << std::endl
        ;
    }

    {
      auto const filename = option->output + ".exr";
      std::cerr << "Exporting " << filename << "..." << std::endl;
      ExportEXR(image.DownSample(option->ssaa), filename);
    }

    {
      auto const filename = option->output + ".png";
      std::cerr << "Exporting " << filename << "..." << std::endl;
      ExportPNG(image.DownSample(option->ssaa), filename, option->exposure);
    }

    return 0;
  }
};

}
}
