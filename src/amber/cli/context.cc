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

#include "amber/cli/context.h"

namespace amber {
namespace cli {

Context::Context(std::size_t n_threads, std::size_t n_iterations) noexcept
: mutex_()
, n_threads_(n_threads)
, n_iterations_(n_iterations)
, n_iterations_started_(0)
, is_expired_(false)
{}

std::size_t
Context::ThreadCount() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return n_threads_;
}

std::size_t
Context::IterationCount() const noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  return n_iterations_started_;
}

bool
Context::Iterate() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (n_iterations_ > 0 && n_iterations_started_ >= n_iterations_) {
    return false;
  }
  if (is_expired_) {
    return false;
  }
  n_iterations_started_++;
  return true;
}

void
Context::Expire() noexcept
{
  std::lock_guard<std::mutex> lock(mutex_);
  is_expired_ = true;
}

}
}
