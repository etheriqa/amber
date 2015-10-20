#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include "camera.h"

namespace amber {
namespace shader {

class ShadingProgress
{
  std::atomic<size_t> m_n_task, m_n_done;
  std::chrono::system_clock::time_point m_begin_time, m_end_time;
  std::vector<std::thread> m_threads;

public:
  ShadingProgress(size_t n_task, std::vector<std::thread>&& threads) :
    m_n_task(n_task),
    m_n_done(0),
    m_begin_time(std::chrono::system_clock::now()),
    m_threads(std::move(threads))
  {}

  ShadingProgress(const ShadingProgress&) = delete;

  ~ShadingProgress()
  {
    for (auto& thread : m_threads) {
      thread.join();
    }
  }

  ShadingProgress& operator=(const ShadingProgress&) = delete;

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

template <typename Acceleration>
struct Shader
{
  using shader_type              = Shader<Acceleration>;
  using acceleration_type        = Acceleration;

  using object_buffer_type       = typename acceleration_type::object_buffer_type;
  using object_type              = typename acceleration_type::object_type;

  using hit_type                 = typename object_type::hit_type;
  using radiant_type             = typename object_type::radiant_type;
  using ray_type                 = typename object_type::ray_type;
  using real_type                = typename object_type::real_type;

  using camera_type              = Camera<radiant_type, real_type>;
  using progress_const_reference = std::shared_ptr<const ShadingProgress>;
  using progress_reference       = std::shared_ptr<ShadingProgress>;
  using progress_type            = ShadingProgress;

  virtual std::string to_string() const = 0;
  virtual progress_const_reference render(const object_buffer_type&, const camera_type&) const = 0;
};

}
}
