#pragma once

#include <algorithm>
#include <vector>
#include "random.h"

namespace amber {

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
  LightSet(const object_buffer_type& objects) {
    for (const auto& object : objects) {
      if (!object.is_emissive()) {
        continue;
      }
      total_power_ += object.power();
      nodes_.emplace_back(l1norm(total_power_), object);
    }
  }

  const radiant_type& total_power() const { return total_power_; }

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
