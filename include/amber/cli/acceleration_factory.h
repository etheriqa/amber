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

#include <memory>

#include "cli/option.h"
#include "core/acceleration/bsp.h"
#include "core/acceleration/bvh.h"
#include "core/acceleration/kdtree.h"
#include "core/acceleration/list.h"

namespace amber {
namespace cli {

class UnknownAccelerationError : public std::runtime_error
{
public:
  UnknownAccelerationError(std::string const& name) noexcept
  : std::runtime_error("Unknown acceleration name: " + name)
  {}
};

template <typename Object>
class AccelerationFactory
{
public:
  using acceleration_ptr = std::shared_ptr<core::Acceleration<Object>>;

private:
  using list_type   = core::acceleration::List<Object>;
  using bsp_type    = core::acceleration::BSP<Object>;
  using kdtree_type = core::acceleration::KDTree<Object>;
  using bvh_type    = core::acceleration::BVH<Object>;

public:
  template <typename InputIterator>
  acceleration_ptr
  operator()(
    InputIterator first,
    InputIterator last,
    CommandLineOption const& option
  ) const
  {
    if (option.acceleration == "list") {
      return std::make_shared<list_type>(first, last);
    }

    if (option.acceleration == "bsp") {
      return std::make_shared<bsp_type>(first, last);
    }

    if (option.acceleration == "kdtree") {
      return std::make_shared<kdtree_type>(first, last);
    }

    if (option.acceleration == "bvh") {
      return std::make_shared<bvh_type>(first, last);
    }

    throw UnknownAccelerationError(option.acceleration);
  }
};

}
}
