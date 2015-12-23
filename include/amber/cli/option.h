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

#include <cmath>
#include <string>

#include <boost/optional.hpp>

namespace amber {
namespace cli {

struct CommandLineOption
{
  std::string acceleration;
  std::double_t alpha;
  std::double_t exposure;
  std::size_t height;
  bool help;
  std::double_t initial_radius;
  std::size_t k;
  std::string output;
  std::double_t p_large;
  std::size_t n_photons;
  std::size_t n_seeds;
  std::string shader;
  std::size_t spp;
  std::size_t ssaa;
  std::size_t n_threads;
  std::size_t time;
  std::size_t width;
};

boost::optional<CommandLineOption>
ParseCommandLineOption(int argc, char** argv);

}
}
