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

#include <mutex>
#include <thread>
#include <vector>

#include <boost/optional.hpp>

namespace amber {
namespace core {

class Context
{
public:
  virtual std::size_t const ThreadCount() const noexcept = 0;

  virtual std::size_t const IterationCount() const noexcept = 0;
  virtual boost::optional<std::size_t> Iteration() noexcept = 0;

  virtual bool IsExpired() const noexcept = 0;
};

class DefaultContext : public Context
{
private:
  std::size_t n_threads_;

  std::mutex mtx_iteration_;
  std::size_t n_iterations_;
  std::size_t iteration_count_;

  bool expired_;

public:
  DefaultContext(
    std::size_t const n_threads,
    std::size_t const n_iterations
  ) noexcept
  : n_threads_(std::max<std::size_t>(1, n_threads))
  , mtx_iteration_()
  , n_iterations_(n_iterations)
  , iteration_count_(0)
  , expired_(false)
  {}

  std::size_t const ThreadCount() const noexcept
  {
    return n_threads_;
  }

  std::size_t const IterationCount() const noexcept
  {
    return iteration_count_;
  }

  boost::optional<std::size_t> Iteration() noexcept
  {
    std::lock_guard<std::mutex> lock(mtx_iteration_);

    if (iteration_count_ >= n_iterations_) {
      return boost::none;
    }

    return ++iteration_count_;
  }

  bool IsExpired() const noexcept
  {
    return expired_;
  }

  void Expire() noexcept
  {
    expired_ = true;
  }
};

inline
void
DoParallel(
  Context const& ctx,
  std::function<void()> f
)
{
  std::vector<std::thread> threads;
  for (std::size_t i = 1; i < ctx.ThreadCount(); i++) {
    threads.emplace_back(f);
  }
  f();
  for (auto& thread : threads) {
    thread.join();
  }
}

inline
void
Iterate(
  Context& ctx,
  std::function<void(std::size_t)> f
)
{
  for (;;) {
    if (ctx.IsExpired()) {
      return;
    }
    auto const i = ctx.Iteration();
    if (!i) {
      return;
    }
    f(*i);
  }
}

inline
void
IterateParallel(
  Context& ctx,
  std::function<void(std::size_t)> f
)
{
  DoParallel(ctx, [&](){ Iterate(ctx, f); });
}

}
}
