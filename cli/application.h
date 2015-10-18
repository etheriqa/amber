#pragma once

#include <getopt.h>
#include <iostream>
#include <string>
#include <thread>
#include "acceleration/kdtree.h"
#include "aperture/circle.h"
#include "aperture/polygon.h"
#include "camera.h"
#include "geometry/vector.h"
#include "image.h"
#include "io/ppm.h"
#include "io/rgbe.h"
#include "lens/pinhole.h"
#include "lens/thin.h"
#include "object.h"
#include "post_process/gamma.h"
#include "post_process/reinhard.h"
#include "radiometry/rgb.h"
#include "render.h"
#include "scene/cornel_box.h"
#include "shader/bidirectional_path_tracing.h"
#include "shader/path_tracing.h"

namespace {

enum class Algorithm {
  pt,
  bdpt,
};

}

namespace amber {
namespace cli {

template <typename RealType>
class Application
{
private:
  using real_type         = RealType;
  using vector3_type      = amber::geometry::Vector3<real_type>;
  using radiant_type      = amber::radiometry::RGB<real_type>;

  using primitive_type    = amber::geometry::primitive::Primitive<real_type>;
  using material_type     = amber::material::Material<radiant_type>;
  using object_type       = amber::Object<primitive_type, material_type>;

  using acceleration_type = amber::acceleration::KDTree<object_type>;

  int argc;
  char **argv;

public:
  Application(int argc, char **argv) : argc(argc), argv(argv) {}

  static void bannar() {
    std::cerr << R"(                   __             )" << std::endl;
    std::cerr << R"(  ____ _____ ___  / /_  ___  _____)" << std::endl;
    std::cerr << R"( / __ `/ __ `__ \/ __ \/ _ \/ ___/)" << std::endl;
    std::cerr << R"(/ /_/ / / / / / / /_/ /  __/ /    )" << std::endl;
    std::cerr << R"(\__,_/_/ /_/ /_/_.___/\___/_/     )" << std::endl;
    std::cerr << std::endl;
  }

  static void help() {
    std::cerr << std::endl;
    std::cerr << "Usage:" << std::endl;
    std::cerr << "    amber --algorithm <algorithm=pt> [--spp <spp>]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Options:" << std::endl;
    std::cerr << "    --algorithm <algorithm=pt>      rendering algorithm to use" << std::endl;
    std::cerr << "    --spp <n>                       samples per pixel" << std::endl;
    std::cerr << std::endl;
    std::cerr << "Algorithms:" << std::endl;
    std::cerr << "    pt          path tracing (spp=1024)" << std::endl;
    std::cerr << "    bdpt        bidirectional path tracing (spp=128)" << std::endl;
  }

  int run() {
    bannar();

    static option options[] = {
      {"help", no_argument, 0, 'h'},
      {"algorithm", required_argument, 0, 'a'},
      {"spp", required_argument, 0, 's'},
      {0, 0, 0, 0},
    };

    bool show_help = false;
    Algorithm algorithm = Algorithm::pt;
    size_t spp = 0;

    int c;
    int option_index = 0;
    for (;;) {
      c = getopt_long(argc, argv, "h", options, &option_index);
      if (c == -1) {
        break;
      }
      switch (c) {
      case 'h':
        show_help = true;
        break;
      case 'a':
        if (std::string(optarg) == "pt") { algorithm = Algorithm::pt; }
        if (std::string(optarg) == "bdpt") { algorithm = Algorithm::bdpt; }
        break;
      case 's':
        spp = std::stoi(std::string(optarg));
        break;
      case '?':
        help();
        return 1;
        break;
      }
    }

    if (show_help) {
      help();
      return 0;
    }

    const auto n_thread = std::thread::hardware_concurrency();
    const auto ssaa_factor = 4;
    const auto width = 512 * ssaa_factor;
    const auto height = 512 * ssaa_factor;
    const auto scene = amber::scene::cornel_box<acceleration_type>();
    amber::shader::Shader<acceleration_type> *shader;
    switch (algorithm) {
    case Algorithm::pt:
      shader = new amber::shader::PathTracing<acceleration_type>(
        n_thread,
        (spp > 0 ? spp : 1024) / ssaa_factor / ssaa_factor
      );
      break;
    case Algorithm::bdpt:
      shader = new amber::shader::BidirectionalPathTracing<acceleration_type>(
        n_thread,
        (spp > 0 ? spp : 128) / ssaa_factor / ssaa_factor
      );
      break;
    }
    const auto lens   = new amber::lens::Pinhole<real_type>();
    const auto image  = new amber::Image<radiant_type>(width, height);
    const auto sensor = new amber::Sensor<radiant_type>(image);
    const auto camera = amber::Camera<radiant_type>(
      lens, sensor,
      vector3_type(0, 0, 4), vector3_type(0, 0, 0), vector3_type(0, 1, 0));

    amber::render(shader, scene, camera);

    amber::post_process::Reinhard<real_type> reinhard;
    amber::post_process::Gamma<real_type> gamma;
    amber::io::export_rgbe("output.hdr", image->down_sample(ssaa_factor));
    amber::io::export_ppm("output.ppm", gamma(reinhard(image->down_sample(ssaa_factor))));

    return 0;
  }
};

}
}