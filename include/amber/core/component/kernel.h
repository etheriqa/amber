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

#include "core/constant.h"

namespace amber {
namespace core {
namespace component {

template <typename T>
class DiskKernel
{
private:
  T radius_;

public:
  explicit DiskKernel(T const& radius) noexcept : radius_(radius) {}

  T const& radius() const { return radius_; }

  T const operator()() const noexcept
  {
    return 1 / (static_cast<T>(kPI) * radius_ * radius_ );
  }
};

template <typename T>
class KernelRadiusSequence
{
private:
  T radius_;
  T alpha_;
  std::size_t i_;

public:
  KernelRadiusSequence(
    T const& initial_radius,
    T const& alpha
  ) noexcept
  : radius_(initial_radius)
  , alpha_(alpha)
  , i_(1)
  {}

  T const operator()() noexcept
  {
    auto const radius = radius_;
    radius_ *= std::sqrt((i_ + alpha_) / (i_ + 1));
    i_++;
    return radius;
  }
};

}
}
}
