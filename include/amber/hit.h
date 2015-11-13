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
#include <limits>

#include "writer.h"
#include "vector.h"

namespace amber {

template <typename RealType>
struct Hit : public Writer
{
  using real_type    = RealType;

  using hit_type     = Hit<real_type>;
  using vector3_type = Vector3<real_type>;

  vector3_type position, normal;
  real_type distance;

  Hit() noexcept
  : position(vector3_type()),
    normal(vector3_type()),
    distance(std::numeric_limits<real_type>::quiet_NaN())
  {}

  Hit(const vector3_type& p, const vector3_type& n, real_type d) noexcept
  : position(p),
    normal(Normalize(n)),
    distance(d)
  {}

  operator bool() const noexcept
  {
    return std::isfinite(distance);
  }

  void Write(std::ostream& os) const noexcept
  {
    if (*this) {
      os
        << "Hit(position=" << position
        << ", normal=" << normal
        << ", distance=" << distance
        << ")";
    } else {
      os << "Hit(none)";
    }
  }
};

}
