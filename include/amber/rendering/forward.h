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

#pragma once

#include <functional>
#include <random>
#include <vector>

#include "amber/postprocess/forward.h"
#include "amber/prelude/forward.h"

namespace amber {
namespace rendering {

enum class SurfaceType;
class Context;
class ObjectPointer;
class Sensor;
template <typename Radiant> class Leading;
template <typename Radiant> class Scatter;
template <typename Radiant> class Scene;
template <typename Radiant> class Algorithm;

template <typename Radiant> class SubpathEvent;
template <typename Radiant> class Subpath;
template <typename Radiant> class BidirectionalPathSamplingBuffer;
template <typename Radiant> class UnifiedPathSamplingBuffer;
template <typename Radiant> class PhotonMap;

template <typename BaseSampler> class PrimarySampleSpacePair;

using prelude::pixel_size_type;
using real_type      = std::float_t;
using path_size_type = std::uint_fast8_t;

using Vector2     = prelude::Vector2<real_type>;
using Vector3     = prelude::Vector3<real_type>;
using UnitVector3 = prelude::UnitVector3<real_type>;
using Ray         = prelude::Ray<real_type>;
using Hit         = prelude::Hit<real_type>;

using prelude::Pixel;
template <typename T> using PixelValue  = prelude::PixelValue<T>;
template <typename T> using Image       = prelude::Image<T>;
template <typename T> using SparseImage = prelude::SparseImage<T>;

using prelude::Sampler;
using MTSampler                = prelude::GenericSampler<std::mt19937_64>;
using MTPrimarySampleSpacePair = PrimarySampleSpacePair<MTSampler>;

template <typename T> using Accumulator = prelude::Accumulator<T>;
template <typename T> using KDTree = prelude::KDTree<real_type, T>;

using postprocess::Monochrome;
using postprocess::RGB;

using MIS = std::function<real_type(const std::vector<real_type>&)>;

}
}
