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

namespace amber {
namespace shader {
namespace framework {

template <typename Object>
class LightSampler {
private:
  using object_type  = Object;
  using radiant_type = typename Object::radiant_type;
  using real_type    = typename Object::real_type;

  struct Node {
    real_type partial_sum_power;
    object_type object;

    Node(real_type partial_sum_power, const object_type& object) noexcept
      : partial_sum_power(partial_sum_power), object(object) {}
  };

  std::vector<Node> nodes_;
  radiant_type total_power_;

public:
  template <typename InputIterator>
  LightSampler(InputIterator first, InputIterator last) noexcept {
    std::for_each(first, last, [&](const auto& object){
      if (!object.isEmissive()) {
        return;
      }
      total_power_ += object.power();
      nodes_.emplace_back(total_power_.sum(), object);
    });
  }

  const radiant_type& total_power() const noexcept { return total_power_; }

  template <typename Random>
  object_type operator()(Random& random) const {
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
}