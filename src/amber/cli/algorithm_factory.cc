// Copyright (c) 2016 TAKAMORI Kaede <etheriqa@gmail.com>
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

#include "amber/cli/algorithm_factory.h"
#include "amber/cli/option.h"
#include "amber/prelude/vector3.h"
#include "amber/rendering/algorithm_bdpt.h"
#include "amber/rendering/algorithm_lt.h"
#include "amber/rendering/algorithm_mppm.h"
#include "amber/rendering/algorithm_pssmlt.h"
#include "amber/rendering/algorithm_pt.h"
#include "amber/rendering/algorithm_sppm.h"
#include "amber/rendering/algorithm_ups.h"

namespace amber {
namespace cli {

std::unique_ptr<rendering::Algorithm<RGB>>
AlgorithmFactory::operator()(const CommandLineOption& option) const
{
  if (option.algorithm == "pt") {
    return rendering::MakeRGBPathTracing();
  }

  if (option.algorithm == "lt") {
    return rendering::MakeRGBLightTracing();
  }

  if (option.algorithm == "bdpt") {
    return rendering::MakeRGBBidirectionalPathTracing();
  }

  if (option.algorithm == "pssmlt") {
    return rendering::MakeRGBPrimarySampleSpaceMLT(
      option.n_discard_samples,
      option.p_large_step
    );
  }

  if (option.algorithm == "sppm") {
    return rendering::MakeRGBStochasticPPM(
      option.initial_radius,
      option.alpha
    );
  }

  if (option.algorithm == "mppm") {
    return rendering::MakeRGBMemorylessPPM(
      option.initial_radius,
      option.alpha
    );
  }

  if (option.algorithm == "ups") {
    return rendering::MakeRGBUnifiedPathSampling(
      option.initial_radius,
      option.alpha
    );
  }

  throw UnknownAlgorithmError(option.algorithm);
}

}
}
