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
#include <tuple>

#include "writer.h"

namespace amber {

template <typename Object>
struct Acceleration : public Writer {
  using object_type = Object;

  using hit_type    = typename Object::hit_type;
  using ray_type    = typename Object::ray_type;

  virtual std::tuple<hit_type, Object>
  Cast(ray_type const&) const noexcept = 0;

  virtual bool
  TestVisibility(
    ray_type const& ray,
    Object const& object
  ) const noexcept
  {
    return std::get<1>(Cast(ray)) == object;
  }

protected:
  using real_type = typename Object::real_type;

  template <typename InputIterator>
  static std::tuple<hit_type, Object>
  Traverse(
    InputIterator first,
    InputIterator last,
    ray_type const& ray,
    real_type t_max
  ) noexcept
  {
    hit_type closest_hit;
    Object closest_object;

    std::for_each(first, last, [&](auto const& object){
      auto const hit = object.Intersect(ray);
      if (hit && hit.distance < t_max) {
        t_max = hit.distance;
        closest_hit = hit;
        closest_object = object;
      }
    });

    return std::make_tuple(closest_hit, closest_object);
  }
};


}
