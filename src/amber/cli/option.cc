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

#include <iostream>
#include <thread>

#include <boost/program_options.hpp>

#include "cli/option.h"

namespace amber {
namespace cli {

CommandLineOption
ParseCommandLineOption(int argc, char** argv)
{
  CommandLineOption option;

  namespace po = boost::program_options;

  po::options_description description("options");
  po::variables_map vm;
  description.add_options()
    ("acceleration",
     po::value<std::string>(&option.acceleration)->default_value("kdtree"),
     "acceleration structure to use (list, bsp, kdtree, bvh)")
    ("alpha",
     po::value<std::double_t>(&option.alpha)->default_value(0.7),
     "a parameter used in the progressive photon mapping algoritms")
    ("exposure",
     po::value<std::double_t>(&option.exposure)->default_value(1),
     "relative exposure value")
    ("height",
     po::value<std::size_t>(&option.height)->default_value(512),
     "image height")
    ("help", "print this message")
    ("initial-radius",
     po::value<std::double_t>(&option.initial_radius)->default_value(0.001),
     "a parameter as a fraction of the scene's bounding box used in the density estimation method")
    ("k",
     po::value<std::size_t>(&option.k)->default_value(16),
     "a parameter of the k-nearest neighbours method used in the photon mapping algorithm")
    ("mutations",
     po::value<std::size_t>(&option.n_mutations)->default_value(262144 * 16),
     "a number of mutations used in the MCMC algorithms")
    ("output",
     po::value<std::string>(&option.output)->default_value("output"),
     "an output filename without an extension")
    ("p-large",
     po::value<std::double_t>(&option.p_large)->default_value(0.5),
     "a large step probability used in the primary sample space method")
    ("passes",
     po::value<std::size_t>(&option.n_passes)->default_value(16),
     "a number of passes used in the progressive photon mapping algorithms")
    ("photons",
     po::value<std::size_t>(&option.n_photons)->default_value(262144),
     "a number of emitted photons")
    ("samples",
     po::value<std::size_t>(&option.n_samples)->default_value(262144),
     "a number of samples used in the light tracing algorithm")
    ("seeds",
     po::value<std::size_t>(&option.n_seeds)->default_value(65536),
     "a number of seed states used in the MCMC algorithms")
    ("shader",
     po::value<std::string>(&option.shader),
     "rendering algorithm to use (pt, lt, bdpt, pssmlt, pm, ppm, sppm)")
    ("spp",
     po::value<std::size_t>(&option.spp)->default_value(16),
     "samples per pixel")
    ("ssaa",
     po::value<std::size_t>(&option.ssaa)->default_value(1),
     "a level of supersampling anti-aliasing")
    ("threads",
     po::value<std::size_t>(&option.n_threads)
       ->default_value(std::thread::hardware_concurrency()),
     "a number of threads")
    ("width",
     po::value<std::size_t>(&option.width)->default_value(512),
     "image width")
    ;

  po::store(po::parse_command_line(argc, argv, description), vm);
  po::notify(vm);

  if (vm.count("help") || !vm.count("shader")) {
    std::cerr << description << std::endl;
    option.help = true;
  }

  return option;
}

}
}