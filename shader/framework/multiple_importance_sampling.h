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
  value_type log_probability_;

public:
  Sample(const Contribution& contribution, value_type log_probability) noexcept
    : contribution_(contribution), log_probability_(log_probability) {}

  const Contribution& contribution() const noexcept {
    return contribution_;
  }

  const value_type& log_probability() const noexcept {
    return log_probability_;
  }

  bool operator<(const Sample& sample) const noexcept {
    return log_probability_ < sample.log_probability_;
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
              return weight +
                std::exp(y.log_probability() - x.log_probability());
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
                std::pow(std::exp(y.log_probability() - x.log_probability()),
                         beta_);
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
