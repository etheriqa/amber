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

#include <algorithm>
#include <numeric>

namespace amber {
namespace core {
namespace component {

class BalanceHeuristic {
public:
  template <typename InputIterator, typename T>
  T operator()(InputIterator first,
               InputIterator last,
               T const& log_p) const noexcept {
    return 1 / std::accumulate(first, last, T(),
      [&](auto const& acc, auto const& log_q){
        return acc + std::exp2(log_q - log_p);
      });
  }
};

template <typename T>
class PowerHeuristic {
private:
  T beta_;

public:
  PowerHeuristic() noexcept : beta_(2) {}
  explicit PowerHeuristic(T const& beta) noexcept : beta_(beta) {}

  template <typename InputIterator>
  T operator()(InputIterator first,
               InputIterator last,
               T const& log_p) const noexcept {
    return 1 / std::accumulate(first, last, T(),
      [&](auto const& acc, auto const& log_q){
        return acc + std::exp2(beta_ * (log_q - log_p));
      });
  }
};

class MaximumHeuristic {
public:
  template <typename InputIterator, typename T>
  T operator()(InputIterator first,
               InputIterator last,
               T const& log_p) const noexcept {
    auto const max_log_q = std::max_element(first, last);
    if (log_p != max_log_q) {
      return T();
    }
    return 1 / static_cast<T>(std::count(first, last, max_log_q));
  }
};

}
}
}
