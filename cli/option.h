/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
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
};

struct Option {
  std::string name           = "output";

  size_t n_thread            = std::thread::hardware_concurrency();

  size_t width               = 512;
  size_t height              = 512;
  size_t ssaa                = 2;

  Algorithm algorithm        = Algorithm::none;

  size_t pt_spp              = 512;

  size_t bdpt_spp            = 128;

  size_t pm_n_photon         = 8388608;
  size_t pm_k_nearest_photon = 16;

  size_t pssmlt_n_seed       = 65536;
  size_t pssmlt_n_mutation   = 33554432;
  double pssmlt_p_large_step = 0.5;
};

}
}
