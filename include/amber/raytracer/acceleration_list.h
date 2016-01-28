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

#include <vector>

#include "amber/raytracer/acceleration.h"

namespace amber {
namespace raytracer {

template <typename T, typename Object>
class List
: public Acceleration<T, Object>
{
public:
  explicit List(std::vector<Object>&& objects) noexcept;

  std::tuple<Hit<T>, const Object*>
  Cast(const Ray<T>& ray, T distance) const noexcept;

private:
  std::vector<Object> objects_;
};



template <typename T, typename Object>
List<T, Object>::List(std::vector<Object>&& objects) noexcept
: objects_(std::move(objects))
{}

template <typename T, typename Object>
std::tuple<Hit<T>, const Object*>
List<T, Object>::Cast(const Ray<T>& ray, T distance) const noexcept
{
  Hit<T> closest_hit;
  const Object* closest_object = nullptr;

  for (const auto& object : objects_) {
    const auto hit = object.Intersect(ray);
    if (hit && hit.Distance() < distance) {
      distance = hit.Distance();
      closest_hit = hit;
      closest_object = &object;
    }
  }

  return std::make_tuple(closest_hit, closest_object);
}

}
}
