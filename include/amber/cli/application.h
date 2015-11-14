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
#include <thread>

#include <boost/program_options.hpp>
#include <opencv2/opencv.hpp>

#include "cli/render.h"
#include "cli/shader_factory.h"
#include "core/acceleration/bsp.h"
#include "core/acceleration/bvh.h"
#include "core/acceleration/kdtree.h"
#include "core/acceleration/list.h"
#include "core/camera.h"
#include "core/lens/thin.h"
#include "core/material/eye.h"
#include "core/object.h"
#include "core/primitive/regular_polygon.h"
#include "core/rgb.h"
#include "core/scene.h"
#include "core/vector.h"
#include "io/rgbe.h"
#include "post_process/filmic.h"
#include "post_process/gamma.h"
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

  using primitive_type     = core::Primitive<real_type>;
  using material_type      = core::Material<radiant_type, real_type>;
  using object_type        = core::Object<primitive_type, material_type>;

  using acceleration_type  = core::acceleration::KDTree<object_type>;

  using scene_type         = core::Scene<acceleration_type>;

  int argc_;
  char **argv_;

public:
  Application(int argc, char **argv) noexcept : argc_(argc), argv_(argv) {}

  int run()
  {
    std::cerr << R"(                   __             )" << std::endl;
    std::cerr << R"(  ____ _____ ___  / /_  ___  _____)" << std::endl;
    std::cerr << R"( / __ `/ __ `__ \/ __ \/ _ \/ ___/)" << std::endl;
    std::cerr << R"(/ /_/ / / / / / / /_/ /  __/ /    )" << std::endl;
    std::cerr << R"(\__,_/_/ /_/ /_/_.___/\___/_/     )" << std::endl;
    std::cerr << std::endl;
    std::cerr << "amber: a global illumination renderer" << std::endl;
    std::cerr << std::endl;

    boost::program_options::options_description description("options");
    boost::program_options::variables_map vm;
    {
      namespace po = boost::program_options;
      description.add_options()
        ("algorithm",
         po::value<std::string>(),
         "rendering algorithm to use (pt, lt, bdpt, pm, pssmlt, ppm, sppm)")
        ("alpha",
         po::value<real_type>()->default_value(0.7),
         "a parameter used in the progressive photon mapping algoritms")
        ("exposure",
         po::value<real_type>()->default_value(1),
         "relative exposure value")
        ("height",
         po::value<size_type>()->default_value(512),
         "image height")
        ("help", "print this message")
        ("k",
         po::value<size_type>()->default_value(16),
         "a parameter of the k-nearest neighbours method used in the photon mapping algorithms")
        ("mutations",
         po::value<size_type>()->default_value(65536),
         "a number of mutations used in the MCMC algorithms")
        ("nl-means",
         po::bool_switch(),
         "use non-local means denoising algorithm")
        ("output",
         po::value<std::string>()->default_value("output"),
         "an output filename without an extension")
        ("p-large",
         po::value<real_type>()->default_value(0.5),
         "a large step probability used in the primary sample space method")
        ("passes",
         po::value<size_type>()->default_value(16),
         "a number of passes used in the progressive photon mapping algorithms")
        ("photons",
         po::value<size_type>()->default_value(65536),
         "a number of emitted photons")
        ("seeds",
         po::value<size_type>()->default_value(65536),
         "a number of seed states used in the MCMC algorithms")
        ("spp",
         po::value<size_type>()->default_value(16),
         "samples per pixel")
        ("ssaa",
         po::value<size_type>()->default_value(1),
         "a level of supersampling anti-aliasing")
        ("threads",
         po::value<size_type>()
           ->default_value(std::thread::hardware_concurrency()),
         "a number of threads")
        ("width",
         po::value<size_type>()->default_value(512),
         "image width")
        ("samples",
         po::value<size_type>()->default_value(65536),
         "a number of samples")
        ;

      try {
        po::store(po::parse_command_line(argc_, argv_, description), vm);
      } catch (po::error_with_option_name const& e) {
        std::cerr << e.what() << std::endl;
        return -1;
      }

      po::notify(vm);
    }

    if (vm.count("help") || !vm.count("algorithm")) {
      std::cerr << description << std::endl;
      return 0;
    }

    ShaderFactory<scene_type>::shader_ptr shader;
    try {
      shader = ShaderFactory<scene_type>()(vm);
    } catch (InvalidAlgorithmError const& e) {
      std::cerr << "invalid algorithm" << std::endl;
      return -1;
    }

    vector3_type const origin(0, 0, 4);
    vector3_type const axis(0, 0, -1);
    vector3_type const up(0, 1, 0);
    size_type const n_blades = 6;
    real_type const radius = 0.05;
    real_type const focus_distance = 4;

    std::vector<object_type> objects;
    scene::cornel_box(std::back_inserter(objects));

    auto const aperture = core::primitive::RegularPolygon<real_type>(
      origin, axis, up, n_blades, radius
    );
    objects.emplace_back(
      &aperture,
      new core::material::Eye<radiant_type, real_type>
    );
    auto const lens = core::lens::Thin<real_type>(focus_distance);
    auto const sensor = core::Sensor<radiant_type, real_type>(
      vm.at("width").as<size_type>() * vm.at("ssaa").as<size_type>(),
      vm.at("height").as<size_type>() * vm.at("ssaa").as<size_type>()
    );
    auto const camera = core::Camera<radiant_type, real_type>(
      sensor, &lens, &aperture, origin, axis, up
    );

    auto const image = render<scene_type>(shader.get(), objects, camera);

    std::cerr << "Total Power = " << image.totalPower() << std::endl;

    {
      auto const filename = vm.at("output").as<std::string>() + ".hdr";
      std::cerr << "Exporting " << filename << "..." << std::endl;
      io::export_rgbe(
        filename,
        image.downSample(vm.at("ssaa").as<size_type>())
      );
    }

    {
      auto const filename = vm.at("output").as<std::string>() + ".png";
      std::cerr << "Exporting " << filename << "..." << std::endl;
      post_process::Filmic<radiant_type>
        filmic(vm.at("exposure").as<std::double_t>());
      post_process::Gamma<radiant_type> gamma;
      auto const ldr_image =
        gamma(filmic(image.downSample(vm.at("ssaa").as<size_type>())));

      cv::Mat mat(ldr_image.height(), ldr_image.width(), CV_8UC3);
      for (size_t i = 0; i < ldr_image.height(); i++) {
        for (size_t j = 0; j < ldr_image.width(); j++) {
          auto& rgb = mat.at<cv::Vec3b>(i, j);
          rgb[0] = ldr_image.at(j, i).b();
          rgb[1] = ldr_image.at(j, i).g();
          rgb[2] = ldr_image.at(j, i).r();
        }
      }
      if (vm.at("nl-means").as<bool>()) {
        auto src = mat;
        cv::fastNlMeansDenoisingColored(src, mat);
      }
      cv::imwrite(filename, mat);
    }

    return 0;
  }
};

}
}
