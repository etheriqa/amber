/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <string>

namespace amber {
namespace cli {

enum class Algorithm {
  none,
  pt,
  bdpt,
  pm,
  pssmlt,
  ppm,
  sppm,
};

struct Option {
  std::string name              = "";

  size_t n_thread               = std::thread::hardware_concurrency();

  size_t width                  = 512;
  size_t height                 = 512;
  size_t ssaa                   = 1;
  double exposure               = 1.0;

  Algorithm algorithm           = Algorithm::none;

  size_t pt_spp                 = 512;

  size_t bdpt_spp               = 64;

  size_t pm_n_photon            = 16777216;
  size_t pm_k_nearest_photon    = 256;

  size_t pssmlt_n_seed          = 65536;
  size_t pssmlt_n_mutation      = 16777216;
  double pssmlt_p_large_step    = 0.5;

  size_t ppm_n_photon           = 4194304;
  size_t ppm_k_nearest_photon   = 256;
  size_t ppm_n_iteration        = 4;
  double ppm_initial_radius     = 0.01;
  double ppm_alpha              = 0.7;

  size_t sppm_n_photons         = 4194304;
  size_t sppm_k_nearest_photons = 256;
  size_t sppm_n_passes          = 4;
  double sppm_initial_radius    = 0.01;
  double sppm_alpha             = 0.7;
};

}
}
