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

#include "amber/rendering/kernel.h"

namespace amber {
namespace rendering {

template <typename T>
class LocalStatistic
{
public:
  explicit LocalStatistic(const T& initial_radius) noexcept;
  explicit LocalStatistic(const LocalStatistic<T>& statistic) noexcept;

  void lock() { mutex_.lock(); }
  void unlock() { mutex_.unlock(); }

  DiskKernel<T> Kernel() const noexcept;
  void ReduceRadius(const T& n_samples, const T& alpha) noexcept;

private:
  std::mutex mutex_;
  T radius_;
  T n_samples_;
};



template <typename T>
LocalStatistic<T>::LocalStatistic(const T& initial_radius) noexcept
: mutex_()
, radius_(initial_radius)
, n_samples_(0)
{}

template <typename T>
LocalStatistic<T>::LocalStatistic(const LocalStatistic<T>& statistic) noexcept
: mutex_()
, radius_(statistic.radius_)
, n_samples_(statistic.n_samples_)
{}

template <typename T>
DiskKernel<T>
LocalStatistic<T>::Kernel() const noexcept
{
  return DiskKernel<T>(radius_);
}

template <typename T>
void
LocalStatistic<T>::ReduceRadius(
  const T& n_samples,
  const T& alpha
) noexcept
{
  if (n_samples_ + n_samples == static_cast<T>(0)) {
    return;
  }

  // XXX note: the following works only for surface rendering
  radius_ *=
    std::sqrt((n_samples + n_samples * alpha) / (n_samples_ + n_samples));
  n_samples_ += n_samples * alpha;
}

}
}
