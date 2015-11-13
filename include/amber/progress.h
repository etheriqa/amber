/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <string>
#include <atomic>

namespace amber {

struct Progress {
  std::string phase;
  std::atomic<size_t> current_phase;
  std::atomic<size_t> total_phase;
  std::atomic<size_t> current_job;
  std::atomic<size_t> total_job;

  Progress(size_t total_phase) noexcept
    : phase("none"),
      current_phase(0), total_phase(total_phase),
      current_job(0), total_job(0) {}
};

}
