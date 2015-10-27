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

namespace {

enum class Algorithm {
  none,
  pt,
  bdpt,
  pm,
  pssmlt,
};

const size_t kSSAA             = 2;
const size_t kWidth            = 512;
const size_t kHeight           = 512;
const size_t kPTSPP            = 512;
const size_t kBDPTSPP          = 128;
const size_t kPMNPhoton        = 8388608;
const size_t kPMNNearestPhoton = 16;
const size_t kPSSMLTNSeed      = 65536;
const size_t kPSSMLTNMutation  = 134217728;
const double kPSSMLTPLargeStep = 0.5;

}

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
      << "    --spp <n>                       samples per pixel (pt, bdpt)" << std::endl
      << "    --photons <n>                   a number of emitting photons (pm)" << std::endl
      << "    --k <n>                         a number of photons used for radiance estimate (pm)" << std::endl
      << "    --mutations <n>                 a number of mutations (pssmlt)" << std::endl
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
      {"algorithm", required_argument, 0, 'a'},
      {"spp", required_argument, 0, 's'},
      {"photons", required_argument, 0, 'p'},
      {"k", required_argument, 0, 'k'},
      {"mutations", required_argument, 0, 'm'},
      {0, 0, 0, 0},
    };

    bool show_help = false;
    Algorithm algorithm = Algorithm::none;
    size_t spp = 0;
    size_t n_photon = 0;
    size_t k_nearest_photon = 0;
    size_t n_mutation = 0;

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
        if (std::string(optarg) == "pm") { algorithm = Algorithm::pm; }
        if (std::string(optarg) == "pssmlt") { algorithm = Algorithm::pssmlt; }
        break;
      case 's':
        spp = std::stoi(std::string(optarg));
        break;
      case 'p':
        n_photon = std::stoi(std::string(optarg));
        break;
      case 'k':
        k_nearest_photon = std::stoi(std::string(optarg));
        break;
      case 'm':
        n_mutation = std::stoi(std::string(optarg));
        break;
      case '?':
        help();
        return 1;
        break;
      }
    }

    if (show_help || algorithm == Algorithm::none) {
      help();
      return 0;
    }

    const auto n_thread = std::thread::hardware_concurrency();
    const auto scene = scene::cornel_box<acceleration_type>();
    shader::Shader<acceleration_type> *shader;
    switch (algorithm) {
    case Algorithm::none:
      throw std::logic_error("invalid algorithm");
      break;
    case Algorithm::pt:
      shader = new shader::PathTracing<acceleration_type>(
        n_thread,
        (spp > 0 ? spp : kPTSPP) / kSSAA / kSSAA
      );
      break;
    case Algorithm::bdpt:
      shader = new shader::BidirectionalPathTracing<acceleration_type>(
        n_thread,
        (spp > 0 ? spp : kBDPTSPP) / kSSAA / kSSAA
      );
      break;
    case Algorithm::pm:
      shader = new shader::PhotonMapping<acceleration_type>(
        n_photon > 0 ? n_photon : kPMNPhoton,
        kPMNNearestPhoton
      );
      break;
    case Algorithm::pssmlt:
      shader = new shader::PrimarySampleSpaceMLT<acceleration_type>(
        n_thread,
        kPSSMLTNSeed,
        (n_mutation > 0 ? n_mutation : kPSSMLTNMutation),
        kPSSMLTPLargeStep
      );
      break;
    }
    const auto lens   = new camera::lens::Pinhole<real_type>();
    const auto image  = new camera::Image<radiant_type>(kWidth * kSSAA, kHeight * kSSAA);
    const auto sensor = camera::Sensor<radiant_type, real_type>(image);
    const auto camera = camera::Camera<radiant_type, real_type>(
      sensor, lens,
      vector3_type(0, 0, 4), vector3_type(0, 0, 0), vector3_type(0, 1, 0));

    render(shader, scene, camera);

    post_process::Filmic<radiant_type> filmic;
    post_process::Gamma<radiant_type> gamma;
    io::export_rgbe("output.hdr", image->downSample(kSSAA));
    io::export_ppm("output.ppm", gamma(filmic(image->downSample(kSSAA))));

    return 0;
  }
};

}
}
