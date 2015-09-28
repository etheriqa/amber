#pragma once

#include <algorithm>
#include <functional>
#include <future>
#include <sstream>
#include <vector>
#include "light_set.h"
#include "random.h"
#include "shader/shader.h"
#include "vector.h"

namespace amber {
namespace shader {

template <class Scene>
class BidirectionalPathTracing : public Shader<Scene>
{
public:
  using shader_type              = Shader<Scene>;

  using camera_type              = typename shader_type::camera_type;
  using flux_type                = typename shader_type::flux_type;
  using hit_type                 = typename shader_type::hit_type;
  using object_type              = typename shader_type::object_type;
  using progress_const_reference = typename shader_type::progress_const_reference;
  using progress_reference       = typename shader_type::progress_reference;
  using progress_type            = typename shader_type::progress_type;
  using ray_type                 = typename shader_type::ray_type;
  using real_type                = typename shader_type::real_type;
  using scene_type               = typename shader_type::scene_type;

  using light_set_type           = LightSet<scene_type>;
  using vector3_type             = Vector3<real_type>;

private:
  struct Event
  {
    object_type object;
    vector3_type position;
    vector3_type normal;
    vector3_type direction_i;
    vector3_type direction_o;
    real_type probability;
    flux_type weight;
  };

  struct Contribution
  {
    real_type probability;
    flux_type power;
  };

  const size_t m_n_thread, m_spp;

public:
  BidirectionalPathTracing(size_t n_thread, size_t spp) :
    m_n_thread(n_thread),
    m_spp(spp)
  {}

  std::string to_string() const
  {
    std::stringstream ss;
    ss << "RealType: " << sizeof(real_type) * 8 << "bit" << std::endl;
    ss << "Shader: BidirectionalPathTracing(n_thread=" << m_n_thread << ", spp=" << m_spp << ")";
    return ss.str();
  }

  progress_const_reference render(const scene_type& scene, const camera_type& camera) const
  {
    // TODO refactor
    std::vector<std::thread> threads;
    std::promise<progress_reference> promise;
    auto future = promise.get_future().share();
    auto current = std::make_shared<std::atomic<size_t>>(0);
    auto pixels = std::make_shared<std::vector<size_t>>(camera.image_pixels());
    std::iota(pixels->begin(), pixels->end(), 0);
    std::shuffle(pixels->begin(), pixels->end(), Random().generator());

    for (size_t i = 0; i < m_n_thread; i++) {
      using namespace std::placeholders;
      threads.push_back(std::thread(std::bind(&BidirectionalPathTracing::process, this, _1, _2, _3, _4, _5), scene, camera, future, current, pixels));
    }

    promise.set_value(std::make_shared<progress_type>(
      m_spp * camera.image_pixels(),
      std::move(threads)
    ));

    return future.get();
  }

private:
  void process(
    const scene_type& scene,
    const camera_type& camera,
    std::shared_future<progress_reference> future,
    std::shared_ptr<std::atomic<size_t>> current,
    std::shared_ptr<std::vector<size_t>> pixels
  ) const
  {
    // TODO refactor
    auto progress = future.get();
    const light_set_type lights(scene);
    Random random;

    for (size_t i = (*current)++; i < camera.image_pixels(); i = (*current)++) {
      auto x = pixels->at(i) % camera.image_width();
      auto y = pixels->at(i) / camera.image_height();
      flux_type power;
      for (size_t j = 0; j < m_spp; j++) {
        power += sample_pixel(scene, lights, camera, x, y, random);
        progress->done(1);
      }
      camera.expose(x, y, power / static_cast<real_type>(m_spp));
    }

    progress->end();
  }

  flux_type sample_pixel(const scene_type& scene, const light_set_type& lights, const camera_type& camera, size_t x, size_t y, Random& random) const
  {
    while (true) {
      const auto light_subpath = sample_light_subpath(scene, lights, random);
      const auto eye_subpath = sample_eye_subpath(scene, camera, x, y, random);
      const auto max_path_length = light_subpath.size() + eye_subpath.size() - 1;

      std::vector<std::vector<Contribution>> contributions(max_path_length);

      for (size_t s = 0; s <= light_subpath.size(); s++) {
        for (size_t t = 0; t <= eye_subpath.size(); t++) {
          Contribution contribution;
          if (s + t < 2) {
            continue;
          } else if (s == 0 && t >= 2) {
            // zero light subpath vertices
            const auto l = eye_subpath[t - 1];
            const auto e = eye_subpath[t - 2];
            if (!l.object.is_emissive()) {
              continue;
            }
            if (dot(e.position - l.position, l.normal) <= 0) {
              continue;
            }
            contribution.power =
              l.object.emittance() / static_cast<real_type>(kPI) *
              l.weight;
            contribution.probability = l.probability;
          } else if (s == 1 && t >= 2) {
            // one light subpath vertex
            const auto l = light_subpath[s - 1];
            const auto e = eye_subpath[t - 1];
            if (e.object.is_specular()) {
              continue;
            }
            if (dot(e.position - l.position, l.normal) <= 0) {
              continue;
            }
            contribution.power =
              l.weight *
              flux_type(1 / static_cast<real_type>(kPI)) *
              geometry_factor(scene, l, e) *
              e.object.bsdf(normalize(l.position - e.position), e.direction_i, e.normal) *
              e.weight;
            contribution.probability = l.probability * e.probability;
          } else if (s >= 2 && t == 0) {
            continue; // TODO zero eye subpath vertices
          } else if (s >= 2 && t == 1) {
            continue; // TODO one eye subpath vertex
          } else if (s == 1 && t == 1) {
            continue; // TODO one light subpath vertex and one eye subpath vertex
          } else {
            const auto& l = light_subpath[s - 1];
            const auto& e = eye_subpath[t - 1];
            if (l.object.is_specular() || e.object.is_specular()) {
              continue;
            }
            contribution.power =
              l.weight *
              l.object.bsdf(l.direction_i, normalize(e.position - l.position), l.normal) *
              geometry_factor(scene, l, e) *
              e.object.bsdf(normalize(l.position - e.position), e.direction_i, e.normal) *
              e.weight;
            contribution.probability = l.probability * e.probability;
          }
          contributions[s + t - 2].push_back(contribution);
        }
      }

      flux_type power;
      for (size_t i = 0; i < max_path_length; i++) {
        for (const auto& c0 : contributions[i]) {
          real_type weight = 0;
          for (const auto& c1 : contributions[i]) {
            weight += std::pow(c1.probability / c0.probability, 2);
          }
          power += c0.power / weight;
        }
      }

      if (!std::isfinite(max(power))) { // FIXME biased
        continue;
      }

      return power;
    }
  }

  std::vector<Event> sample_light_subpath(const scene_type& scene, const light_set_type& lights, Random& random) const
  {
    // s = 0
    const auto object = lights.sample(random);
    const auto sample = object.sample_initial_ray(random);
    const auto area_probability = 1 / lights.total_area();        // FIXME
    const auto psa_probability = static_cast<real_type>(1 / kPI); // FIXME

    Event event;
    event.object      = object;
    event.position    = sample.ray.origin;
    event.normal      = sample.normal;
    event.direction_i = vector3_type();
    event.direction_o = sample.ray.direction;
    event.probability = area_probability;
    event.weight      = object.emittance() / area_probability;

    // s > 0
    return extend_subpath(scene, event, flux_type(psa_probability), psa_probability, random);
  }

  std::vector<Event> sample_eye_subpath(const scene_type& scene, const camera_type& camera, size_t x, size_t y, Random& random) const
  {
    // t = 0
    const auto importance = flux_type(1);                         // FIXME
    const auto sample = camera.sample_initial_ray(x, y, random);
    const auto area_probability = static_cast<real_type>(1);      // FIXME
    const auto psa_probability = static_cast<real_type>(1 / kPI); // FIXME


    Event event;
    event.object      = object_type(nullptr, nullptr);
    event.position    = sample.ray.origin;
    event.normal      = sample.normal;
    event.direction_i = vector3_type();
    event.direction_o = sample.ray.direction;
    event.probability = area_probability;
    event.weight      = importance / area_probability;

    // t > 0
    return extend_subpath(scene, event, flux_type(psa_probability), psa_probability, random);
  }

  std::vector<Event> extend_subpath(const scene_type& scene, Event event, flux_type bsdf, real_type probability, Random& random) const
  {
    std::vector<Event> subpath;
    subpath.reserve(16);
    subpath.push_back(event);

    ray_type ray(event.position, event.direction_o);
    hit_type hit;
    object_type object;

    while (true) {
      std::tie(hit, object) = scene.intersect(ray);
      if (!hit) {
        break;
      }

      const auto sample = object.sample_scattering(-ray.direction, hit.normal, random);

      const auto geometry_factor = std::abs(dot(ray.direction, event.normal) * dot(ray.direction, hit.normal)) / (hit.distance * hit.distance);

      event.object      = object;
      event.position    = hit.position;
      event.normal      = hit.normal;
      event.direction_i = -ray.direction;
      event.direction_o = sample.direction_o;
      event.probability *= probability * geometry_factor;
      event.weight      *= bsdf / probability;
      subpath.push_back(event);

      ray = ray_type(hit.position, sample.direction_o);
      bsdf = sample.bsdf;
      probability = sample.psa_probability;

      const auto p_russian_roulette = max(bsdf / probability);
      if (random.uniform<real_type>() >= p_russian_roulette) {
        break;
      }
      probability *= std::min(static_cast<real_type>(1), p_russian_roulette);
    }

    return subpath;
  }

  real_type geometry_factor(const scene_type& scene, const Event& l, const Event& e) const noexcept
  {
    if (!is_visible(scene, l, e)) {
      return 0;
    }

    const auto direction = normalize(e.position - l.position);

    return std::abs(dot(direction, l.normal) * dot(direction, e.normal)) / norm2(e.position - l.position);
  }

  bool is_visible(const scene_type& scene, const Event& l, const Event& e) const noexcept
  {
    object_type object;
    std::tie(std::ignore, object) = scene.intersect(ray_type(l.position, e.position - l.position));
    return object.shape == e.object.shape && object.material == e.object.material;
  }
};

}
}
