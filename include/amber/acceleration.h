/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

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
