#pragma once

#include <atomic>
#include <chrono>
#include <thread>
#include "camera.h"

namespace amber {
namespace shader {

class ShadingProgress
{
  std::atomic<size_t>                   m_n_task,
                                        m_n_done;
  std::chrono::system_clock::time_point m_begin_time,
                                        m_end_time;
  std::vector<std::thread>              m_threads;

public:
  ShadingProgress(size_t n_task, std::vector<std::thread>&& threads) :
    m_n_task(n_task),
    m_n_done(0),
    m_begin_time(std::chrono::system_clock::now()),
    m_threads(std::move(threads))
  {}

  ~ShadingProgress()
  {
    for (auto& thread : m_threads) {
      thread.join();
    }
  }

  size_t n_task() const
  {
    return m_n_task.load();
  }

  size_t n_done() const
  {
    return m_n_done.load();
  }

  auto begin_time() const
  {
    return m_begin_time;
  }

  auto end_time() const
  {
    return m_end_time;
  }

  bool is_done() const
  {
    return n_task() == n_done();
  }

  auto duration() const
  {
    return m_end_time - m_begin_time;
  }

  void done(size_t n)
  {
    m_n_done += n;
  }

  void end()
  {
    m_end_time = std::chrono::system_clock::now();
  }
};

}
}
