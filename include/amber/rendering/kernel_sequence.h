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

#include <cmath>
#include <mutex>

namespace amber {
namespace rendering {

template <typename T, typename Kernel>
class KernelSequence
{
public:
  KernelSequence(const T& initial_radius, const T& alpha) noexcept;

  Kernel operator()() noexcept;

private:
  std::mutex mutex_;
  T radius_;
  T alpha_;
  std::size_t i_;
};



template <typename T, typename Kernel>
KernelSequence<T, Kernel>::KernelSequence(
  const T& initial_radius,
  const T& alpha
) noexcept
: mutex_()
, radius_(initial_radius)
, alpha_(alpha)
, i_(1)
{}

template <typename T, typename Kernel>
Kernel
KernelSequence<T, Kernel>::operator()() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto radius = radius_;
  radius_ *= std::sqrt((i_ + alpha_) / (i_ + 1));
  i_++;
  return Kernel(radius);
}

}
}
