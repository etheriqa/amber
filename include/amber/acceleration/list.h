/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

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
