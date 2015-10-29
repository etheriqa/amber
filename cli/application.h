/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <iostream>
#include <string>
#include <thread>

#include "getopt.h"

#include "acceleration/kdtree.h"
#include "camera/aperture/circle.h"
#include "camera/aperture/polygon.h"
#include "camera/camera.h"
#include "camera/image.h"
#include "camera/lens/pinhole.h"
#include "camera/lens/thin.h"
#include "cli/option.h"
#include "cli/render.h"
#include "geometry/vector.h"
#include "io/ppm.h"
#include "io/rgbe.h"
#include "object/object.h"
#include "post_process/filmic.h"
#include "post_process/gamma.h"
#include "radiometry/rgb.h"
#include "scene/cornel_box.h"
#include "shader/bidirectional_path_tracing.h"
#include "shader/path_tracing.h"
#include "shader/photon_mapping.h"
#include "shader/primary_sample_space_mlt.h"

namespace amber {
namespace cli {

class Application
{
private:
  using real_type          = std::double_t;
  using vector3_type       = geometry::Vector3<real_type>;

  using radiant_value_type = std::float_t;
  using radiant_type       = radiometry::RGB<radiant_value_type>;

  using primitive_type     = geometry::primitive::Primitive<real_type>;
  using material_type      = material::Material<radiant_type, real_type>;
  using object_type        = object::Object<primitive_type, material_type>;

  using acceleration_type  = amber::acceleration::KDTree<object_type>;

  int argc;
  char **argv;

public:
  Application(int argc, char **argv) : argc(argc), argv(argv) {}

  static void bannar() {
    std::cerr
      << R"(                   __             )" << std::endl
      << R"(  ____ _____ ___  / /_  ___  _____)" << std::endl
      << R"( / __ `/ __ `__ \/ __ \/ _ \/ ___/)" << std::endl
      << R"(/ /_/ / / / / / / /_/ /  __/ /    )" << std::endl
      << R"(\__,_/_/ /_/ /_/_.___/\___/_/     )" << std::endl
      << std::endl;
  }

  static void help() {
    std::cerr
      << std::endl
      << "Usage:" << std::endl
      << "    amber --algorithm <algorithm>" << std::endl
      << std::endl
      << "Options:" << std::endl
      << "    --algorithm <algorithm=pt>      rendering algorithm to use" << std::endl
      << "    --k <n>                         a number of photons used for radiance estimate (pm)" << std::endl
      << "    --mutations <n>                 a number of mutations (pssmlt)" << std::endl
      << "    --name <name=output>" << std::endl
      << "    --photons <n>                   a number of emitting photons (pm)" << std::endl
      << "    --spp <n>                       samples per pixel (pt, bdpt)" << std::endl
      << "    --threads <n>" << std::endl
      << std::endl
      << "Algorithms:" << std::endl
      << "    pt          Path Tracing" << std::endl
      << "    bdpt        Bidirectional Path Tracing" << std::endl
      << "    pm          Photon Mapping" << std::endl
      << "    pssmlt      Primary Sample Space MLT" << std::endl
      << std::endl;
  }

  int run() {
    bannar();

    static option options[] = {
      {"help", no_argument, 0, 'h'},
      {"name", required_argument, 0, 'n'},
      {"threads", required_argument, 0, 't'},
      {"algorithm", required_argument, 0, 'a'},
      {"spp", required_argument, 0, 's'},
      {"photons", required_argument, 0, 'p'},
      {"k", required_argument, 0, 'k'},
      {"mutations", required_argument, 0, 'm'},
      {0, 0, 0, 0},
    };

    bool show_help = false;
    Option option;

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
      case 'n':
        option.name = std::string(optarg);
        break;
      case 't':
        option.n_thread = std::stoull(std::string(optarg));
        break;
      case 'a':
        if (std::string(optarg) == "pt") {
          option.algorithm = Algorithm::pt;
        }
        if (std::string(optarg) == "bdpt") {
          option.algorithm = Algorithm::bdpt;
        }
        if (std::string(optarg) == "pm") {
          option.algorithm = Algorithm::pm;
        }
        if (std::string(optarg) == "pssmlt") {
          option.algorithm = Algorithm::pssmlt;
        }
        break;
      case 's':
        option.pt_spp = std::stoull(std::string(optarg));
        option.bdpt_spp = std::stoull(std::string(optarg));
        break;
      case 'p':
        option.pm_n_photon = std::stoull(std::string(optarg));
        break;
      case 'k':
        option.pm_k_nearest_photon = std::stoull(std::string(optarg));
        break;
      case 'm':
        option.pssmlt_n_mutation = std::stoull(std::string(optarg));
        break;
      case '?':
        help();
        return 1;
        break;
      }
    }

    if (show_help || option.algorithm == Algorithm::none) {
      help();
      return 0;
    }

    const auto scene = scene::cornel_box<acceleration_type>();
    shader::Shader<acceleration_type> *shader;
    switch (option.algorithm) {
    case Algorithm::none:
      throw std::logic_error("invalid algorithm");
      break;
    case Algorithm::pt:
      shader = new shader::PathTracing<acceleration_type>(
        option.n_thread,
        option.pt_spp / option.ssaa / option.ssaa
      );
      break;
    case Algorithm::bdpt:
      shader = new shader::BidirectionalPathTracing<acceleration_type>(
        option.n_thread,
        option.bdpt_spp / option.ssaa / option.ssaa
      );
      break;
    case Algorithm::pm:
      shader = new shader::PhotonMapping<acceleration_type>(
        option.pm_n_photon,
        option.pm_k_nearest_photon
      );
      break;
    case Algorithm::pssmlt:
      shader = new shader::PrimarySampleSpaceMLT<acceleration_type>(
        option.n_thread,
        option.pssmlt_n_seed,
        option.pssmlt_n_mutation,
        option.pssmlt_p_large_step
      );
      break;
    }
    const auto aperture = new camera::aperture::Polygon<real_type>(8, 0.05);
    const auto lens = new camera::lens::Thin<real_type>(aperture, 4);
    const auto image = new camera::Image<radiant_type>(
      option.width * option.ssaa,
      option.height * option.ssaa
    );
    const auto sensor = camera::Sensor<radiant_type, real_type>(image);
    const auto camera = camera::Camera<radiant_type, real_type>(
      sensor, lens,
      vector3_type(0, 0, 4), vector3_type(0, 0, 0), vector3_type(0, 1, 0));

    render(shader, scene, camera);

    post_process::Filmic<radiant_type> filmic;
    post_process::Gamma<radiant_type> gamma;
    io::export_rgbe(option.name + ".hdr", image->downSample(option.ssaa));
    io::export_ppm(option.name + ".ppm", gamma(filmic(image->downSample(option.ssaa))));

    return 0;
  }
};

}
}
