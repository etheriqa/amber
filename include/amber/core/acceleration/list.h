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

#include <limits>
#include <numeric>
#include <vector>

#include "core/acceleration.h"

namespace amber {
namespace core {
namespace acceleration {

template <typename Object>
class List : public Acceleration<Object>
{
private:
  using typename Acceleration<Object>::aabb_type;
  using typename Acceleration<Object>::hit_type;
  using typename Acceleration<Object>::ray_type;

  using real_type = typename Object::real_type;

  std::vector<Object> objects_;
  aabb_type bb_;

public:
  template <typename InputIterator>
  List(InputIterator first, InputIterator last)
  : objects_(first, last),
    bb_(std::accumulate(
      first,
      last,
      aabb_type(),
      [](auto const& acc, auto const& object){
        return acc + object.BoundingBox();
      }
    ))
  {}

  void Write(std::ostream& os) const noexcept
  {
    os << "List()";
  }

  aabb_type const&
  BoundingBox() const noexcept
  {
    return bb_;
  }

  std::tuple<hit_type, Object>
  Cast(const ray_type& ray) const noexcept
  {
    return Acceleration<Object>::Traverse(
      objects_.begin(),
      objects_.end(),
      ray,
      std::numeric_limits<real_type>::max()
    );
  }
};

}
}
}
