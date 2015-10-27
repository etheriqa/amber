/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <limits>
#include <vector>

#include "acceleration/acceleration.h"

namespace amber {
namespace acceleration {

template <typename Object, typename ObjectBuffer = std::vector<Object>>
class List : public Acceleration<Object>
{
public:
  using acceleration_type  = Acceleration<Object>;
  using object_buffer_type = ObjectBuffer;

  using hit_type           = typename acceleration_type::hit_type;
  using object_type        = typename acceleration_type::object_type;
  using ray_type           = typename acceleration_type::ray_type;

private:
  using real_type = typename object_type::real_type;

  ObjectBuffer m_objects;

public:
  explicit List(const object_buffer_type& objects) : m_objects(objects) {}

  void write(std::ostream& os) const noexcept {
    os << "List()";
  }

  std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
  {
    return acceleration_type::traverse(m_objects.begin(), m_objects.end(), ray, std::numeric_limits<real_type>::max());
  }
};

}
}
