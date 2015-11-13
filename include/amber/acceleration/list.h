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
#include <vector>

#include "acceleration.h"

namespace amber {
namespace acceleration {

template <typename Object>
class List : public Acceleration<Object>
{
public:
  using object_type = Object;

  using hit_type    = typename Object::hit_type;
  using ray_type    = typename Object::ray_type;

private:
  using real_type   = typename Object::real_type;

  std::vector<Object> objects_;

public:
  template <typename InputIterator>
  List(InputIterator first, InputIterator last) : objects_(first, last) {}

  void Write(std::ostream& os) const noexcept
  {
    os << "List()";
  }

  std::tuple<hit_type, Object>
  Cast(const ray_type& ray) const noexcept
  {
    return
      Acceleration<Object>::Traverse(objects_.begin(),
                                     objects_.end(),
                                     ray,
                                     std::numeric_limits<real_type>::max());
  }
};

}
}
