/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>

namespace amber {
namespace shader {
namespace framework {

template <typename Contribution>
class Sample {
private:
  using value_type = typename Contribution::value_type;

  Contribution contribution_;
  value_type probability_;

public:
  Sample(const Contribution& contribution, value_type probability) noexcept
    : contribution_(contribution), probability_(probability) {}

  const Contribution& contribution() const noexcept { return contribution_; }
  const value_type& probability() const noexcept { return probability_; }

  bool operator<(const Sample& sample) const noexcept {
    return probability_ < sample.probability_;
  }
};

template <typename Contribution>
class BalanceHeuristic {
private:
  using value_type = typename Contribution::value_type;

public:
  template <typename InputIterator>
  Contribution operator()(InputIterator first,
                          InputIterator last) const noexcept {
    return std::accumulate(first, last, Contribution(),
      [&](const auto& contribution, const auto& x){
        return contribution + x.contribution() /
          std::accumulate(first, last, value_type(),
            [&](const auto& weight, const auto& y){
              return weight + y.probability() / x.probability();
            });
      });
  }
};

template <typename Contribution>
class PowerHeuristic {
private:
  using value_type = typename Contribution::value_type;

  value_type beta_;

public:
  PowerHeuristic() noexcept : beta_(2) {}

  explicit PowerHeuristic(value_type beta) noexcept : beta_(beta) {}

  template <typename InputIterator>
  Contribution operator()(InputIterator first,
                          InputIterator last) const noexcept {
    return std::accumulate(first, last, Contribution(),
      [&](const auto& contribution, const auto& x){
        return contribution + x.contribution() /
          std::accumulate(first, last, value_type(),
            [&](const auto& weight, const auto& y){
              return weight +
                std::pow(y.probability() / x.probability(), beta_);
            });
      });
  }
};

template <typename Contribution>
class MaximumHeuristic {
public:
  template <typename InputIterator>
  Contribution operator()(InputIterator first,
                          InputIterator last) const noexcept {
    if (std::distance(first, last) == 0) {
      return Contribution();
    }
    return std::max_element(first, last)->contribution();
  }
};

}
}
}
