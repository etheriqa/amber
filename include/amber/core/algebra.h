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
#include <utility>

#include <boost/optional.hpp>

namespace amber {
namespace core {

template <typename RealType>
boost::optional<std::tuple<RealType, RealType>>
SolveQuadratic(RealType a, RealType b, RealType c) noexcept
{
  const auto d = b * b - 4 * a * c;
  if (d < 0) {
    return boost::none;
  }

  const auto sqrt_d = std::sqrt(d);
  auto alpha = - b - sqrt_d;
  auto beta = - b + sqrt_d;
  if (std::abs(alpha) < std::abs(beta)) {
    alpha = c / beta * 2;
    beta /= 2 * a;
  } else {
    beta = c / alpha * 2;
    alpha /= 2 * a;
  }

  return std::make_tuple(alpha, beta);
}

}
}
