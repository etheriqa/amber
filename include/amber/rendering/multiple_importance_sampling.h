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

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace amber {
namespace rendering {

class BalanceHeuristic
{
public:
  template <typename T>
  const T operator()(const std::vector<T>& probabilities) const noexcept;
};

template <typename U>
class PowerHeuristic
{
public:
  PowerHeuristic() noexcept;
  explicit PowerHeuristic(const U& exponent) noexcept;

  template <typename T>
  const T operator()(const std::vector<T>& probabilities) const noexcept;

private:
  U exponent_;
};

class MaximumHeuristic
{
public:
  template <typename T>
  const T operator()(const std::vector<T>& probabilities) const noexcept;
};



template <typename T>
const T
BalanceHeuristic
::operator()(const std::vector<T>& probabilities) const noexcept
{
  return
    static_cast<T>(1) /
    std::accumulate(
      probabilities.begin(),
      probabilities.end(),
      static_cast<T>(0)
    );
}

template <typename U>
PowerHeuristic<U>::PowerHeuristic() noexcept
: exponent_(2)
{}

template <typename U>
PowerHeuristic<U>::PowerHeuristic(const U& exponent) noexcept
: exponent_(exponent)
{}

template <typename U>
template <typename T>
const T
PowerHeuristic<U>
::operator()(const std::vector<T>& probabilities) const noexcept
{
  return
    static_cast<T>(1) /
    std::accumulate(
      probabilities.begin(),
      probabilities.end(),
      static_cast<T>(0),
      [&](const auto& acc, const auto& p){
        return acc + std::pow(p, exponent_);
      }
    );
}

template <typename T>
const T
MaximumHeuristic
::operator()(const std::vector<T>& probabilities) const noexcept
{
  if (*std::max_element(probabilities.begin(), probabilities.end()) > 1) {
    return 0;
  }

  return
    static_cast<T>(1) /
    static_cast<T>(std::count(
      probabilities.begin(),
      probabilities.end(),
      static_cast<T>(1)
    ));
}

}
}
