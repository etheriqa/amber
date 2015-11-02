/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <numeric>

namespace amber {
namespace shader {
namespace framework {

class BalanceHeuristic {
public:
  template <typename InputIterator, typename T>
  T operator()(InputIterator first,
               InputIterator last,
               T const& log_p) const noexcept {
    return 1 / std::accumulate(first, last, T(),
      [&](auto const& acc, auto const& log_q){
        return acc + std::exp2(log_q - log_p);
      });
  }
};

template <typename T>
class PowerHeuristic {
private:
  T beta_;

public:
  PowerHeuristic() noexcept : beta_(2) {}
  explicit PowerHeuristic(T const& beta) noexcept : beta_(beta) {}

  template <typename InputIterator>
  T operator()(InputIterator first,
               InputIterator last,
               T const& log_p) const noexcept {
    return 1 / std::accumulate(first, last, T(),
      [&](auto const& acc, auto const& log_q){
        return acc + std::exp2(beta_ * (log_q - log_p));
      });
  }
};

class MaximumHeuristic {
public:
  template <typename InputIterator, typename T>
  T operator()(InputIterator first,
               InputIterator last,
               T const& log_p) const noexcept {
    auto const max_log_q = std::max_element(first, last);
    if (log_p != max_log_q) {
      return T();
    }
    return 1 / static_cast<T>(std::count(first, last, max_log_q));
  }
};

}
}
}
