/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <vector>
#include "random.h"

namespace amber {
namespace object {

template <typename Acceleration>
class LightSet {
public:
  using acceleration_type  = Acceleration;

  using object_buffer_type = typename acceleration_type::object_buffer_type;
  using object_type        = typename acceleration_type::object_type;
  using radiant_type       = typename acceleration_type::object_type::radiant_type;
  using real_type          = typename acceleration_type::object_type::real_type;

private:
  struct Node {
    real_type partial_sum_power;
    object_type object;

    Node(real_type partial_sum_power, const object_type& object) :
      partial_sum_power(partial_sum_power), object(object) {}
  };

  std::vector<Node> nodes_;
  radiant_type total_power_;

public:
  LightSet(const object_buffer_type& objects) noexcept {
    for (const auto& object : objects) {
      if (!object.isEmissive()) {
        continue;
      }
      total_power_ += object.power();
      nodes_.emplace_back(total_power_.sum(), object);
    }
  }

  const radiant_type& total_power() const noexcept { return total_power_; }

  object_type sample(Random& random) const {
    const auto it = std::upper_bound(
      nodes_.begin(), nodes_.end(),
      Node(random.uniform(nodes_.back().partial_sum_power), object_type()),
      [](const auto& a, const auto& b){
        return a.partial_sum_power < b.partial_sum_power;
      }
    );
    return it->object;
  }
};

}
}
