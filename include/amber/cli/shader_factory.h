/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <memory>

#include <boost/program_options.hpp>

#include "shader/bidirectional_path_tracing.h"
#include "shader/light_tracing.h"
#include "shader/path_tracing.h"
#include "shader/photon_mapping.h"
#include "shader/primary_sample_space_mlt.h"
#include "shader/progressive_photon_mapping.h"
#include "shader/stochastic_progressive_photon_mapping.h"

namespace amber {
namespace cli {

class InvalidAlgorithmError : public std::runtime_error
{
public:
  InvalidAlgorithmError() noexcept : std::runtime_error("Invalid algorithm") {}
};

template <typename Scene>
class ShaderFactory
{
public:
  using shader_ptr = std::shared_ptr<Shader<Scene>>;

private:
  using real_type = std::double_t;
  using size_type = std::size_t;

public:
  shader_ptr operator()(boost::program_options::variables_map const& vm) const
  {
    auto const algorithm = vm.at("algorithm").as<std::string>();

    if (algorithm == "pt") {
      return
        std::make_shared<shader::PathTracing<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("spp").as<size_type>()
        );
    }

    if (algorithm == "lt") {
      return
        std::make_shared<shader::LightTracing<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("samples").as<size_type>()
        );
    }

    if (algorithm == "bdpt") {
      return
        std::make_shared<shader::BidirectionalPathTracing<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("spp").as<size_type>()
        );
    }

    if (algorithm == "pm") {
      return
        std::make_shared<shader::PhotonMapping<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("photons").as<size_type>(),
          vm.at("k").as<size_type>()
        );
    }

    if (algorithm == "pssmlt") {
      return
        std::make_shared<shader::PrimarySampleSpaceMLT<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("seeds").as<size_type>(),
          vm.at("mutations").as<size_type>(),
          vm.at("p-large").as<real_type>()
        );
    }

    if (algorithm == "ppm") {
      return
        std::make_shared<shader::ProgressivePhotonMapping<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("photons").as<size_type>(),
          vm.at("k").as<size_type>(),
          vm.at("passes").as<size_type>(),
          0.01,
          vm.at("alpha").as<real_type>()
        );
    }

    if (algorithm == "sppm") {
      return
        std::make_shared<shader::StochasticProgressivePhotonMapping<Scene>>(
          vm.at("threads").as<size_type>(),
          vm.at("photons").as<size_type>(),
          vm.at("k").as<size_type>(),
          vm.at("passes").as<size_type>(),
          0.01,
          vm.at("alpha").as<real_type>()
        );
    }

    throw InvalidAlgorithmError();
  }
};

}
}
