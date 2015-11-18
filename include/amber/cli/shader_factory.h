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

#include <memory>
#include <string>

#include <boost/program_options.hpp>

#include "core/shader/bidirectional_path_tracing.h"
#include "core/shader/light_tracing.h"
#include "core/shader/path_tracing.h"
#include "core/shader/photon_mapping.h"
#include "core/shader/primary_sample_space_mlt.h"
#include "core/shader/progressive_photon_mapping.h"
#include "core/shader/stochastic_progressive_photon_mapping.h"

namespace amber {
namespace cli {

class UnknownShaderError : public std::runtime_error
{
public:
  UnknownShaderError(std::string const& name) noexcept
  : std::runtime_error("Unknown shader name: " + name)
  {}
};

template <typename Object>
class ShaderFactory
{
public:
  using shader_ptr = std::shared_ptr<core::Shader<Object>>;

private:
  using real_type = std::double_t;
  using size_type = std::size_t;

  using pt_type     = core::shader::PathTracing<Object>;
  using lt_type     = core::shader::LightTracing<Object>;
  using bdpt_type   = core::shader::BidirectionalPathTracing<Object>;
  using pssmlt_type = core::shader::PrimarySampleSpaceMLT<Object>;
  using pm_type     = core::shader::PhotonMapping<Object>;
  using ppm_type    = core::shader::ProgressivePhotonMapping<Object>;
  using sppm_type   = core::shader::StochasticProgressivePhotonMapping<Object>;

public:
  shader_ptr
  operator()(boost::program_options::variables_map const& vm) const
  {
    auto const shader = vm.at("shader").as<std::string>();

    if (shader == "pt") {
      return std::make_shared<pt_type>(
        vm.at("threads").as<size_type>(),
        vm.at("spp").as<size_type>()
      );
    }

    if (shader == "lt") {
      return std::make_shared<lt_type>(
        vm.at("threads").as<size_type>(),
        vm.at("samples").as<size_type>()
      );
    }

    if (shader == "bdpt") {
      return std::make_shared<bdpt_type>(
        vm.at("threads").as<size_type>(),
        vm.at("spp").as<size_type>()
      );
    }

    if (shader == "pssmlt") {
      return std::make_shared<pssmlt_type>(
        vm.at("threads").as<size_type>(),
        vm.at("seeds").as<size_type>(),
        vm.at("mutations").as<size_type>(),
        vm.at("p-large").as<real_type>()
      );
    }

    if (shader == "pm") {
      return std::make_shared<pm_type>(
        vm.at("threads").as<size_type>(),
        vm.at("photons").as<size_type>(),
        vm.at("k").as<size_type>()
      );
    }

    if (shader == "ppm") {
      return std::make_shared<ppm_type>(
        vm.at("threads").as<size_type>(),
        vm.at("photons").as<size_type>(),
        vm.at("passes").as<size_type>(),
        vm.at("initial-radius").as<real_type>(),
        vm.at("alpha").as<real_type>()
      );
    }

    if (shader == "sppm") {
      return std::make_shared<sppm_type>(
        vm.at("threads").as<size_type>(),
        vm.at("photons").as<size_type>(),
        vm.at("passes").as<size_type>(),
        vm.at("initial-radius").as<real_type>(),
        vm.at("alpha").as<real_type>()
      );
    }

    throw UnknownShaderError(shader);
  }
};

}
}
