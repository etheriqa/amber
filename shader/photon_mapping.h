#pragma once

#include <algorithm>
#include <future>
#include <iterator>
#include <sstream>
#include <vector>
#include "geometry/aabb.h"
#include "light_set.h"
#include "shader/shader.h"

namespace amber {
namespace shader {

template <typename Acceleration>
class PhotonMapping : public Shader<Acceleration>
{
public:
  using shader_type              = Shader<Acceleration>;

  using acceleration_type        = typename shader_type::acceleration_type;
  using camera_type              = typename shader_type::camera_type;
  using hit_type                 = typename shader_type::hit_type;
  using object_buffer_type       = typename shader_type::object_buffer_type;
  using object_type              = typename shader_type::object_type;
  using progress_const_reference = typename shader_type::progress_const_reference;
  using progress_type            = typename shader_type::progress_type;
  using radiant_type             = typename shader_type::radiant_type;
  using ray_type                 = typename shader_type::ray_type;
  using real_type                = typename shader_type::real_type;

  using aabb_type                = geometry::AABB<real_type>;
  using light_set_type           = LightSet<acceleration_type>;
  using vector3_type             = geometry::Vector3<real_type>;

private:
  enum struct Axis {
    x,
    y,
    z,
  };

  struct Photon {
    vector3_type position;
    vector3_type direction;
    radiant_type power;
    Axis axis;
  };

  struct PhotonComparator {
    vector3_type point;
    explicit PhotonComparator(const vector3_type& point) : point(point) {}
    bool operator()(const Photon& a, const Photon& b) const {
      return
        (a.position - point).squaredLength()
        < (b.position - point).squaredLength();
    }
  };

  struct PhotonMap {
    std::vector<Photon> photons_;

    PhotonMap(std::vector<Photon> photons) : photons_(photons.size()) {
      buildPhotonMap(photons.begin(), photons.end(), 0);
    }

    template <typename RandomAccessIterator>
    void buildPhotonMap(RandomAccessIterator first,
                        RandomAccessIterator last,
                        size_t pos) {
      if (pos >= photons_.size()) {
        return;
      }
      aabb_type aabb;
      std::for_each(first, last, [&](const auto& photon){
        aabb += aabb_type(photon.position);
      });
      const auto x = aabb.max.x() - aabb.min.x();
      const auto y = aabb.max.y() - aabb.min.y();
      const auto z = aabb.max.z() - aabb.min.z();
      Axis axis;
      if (x > y && x > z) {
        axis = Axis::x;
      } else if (y > z) {
        axis = Axis::y;
      } else {
        axis = Axis::z;
      }
      std::sort(first, last, [&](const auto& a, const auto& b){
          switch (axis) {
          case Axis::x:
            return a.position.x() < b.position.x();
          case Axis::y:
            return a.position.y() < b.position.y();
          case Axis::z:
            return a.position.z() < b.position.z();
          }
      });
      const auto size = std::distance(first, last);
      size_t left_size = 0, right_size = 0;
      while (left_size + right_size + 1 < size) {
        left_size = std::min(size - 1 - right_size, (left_size << 1) + 1);
        right_size = std::min(size - 1 - left_size, (right_size << 1) + 1);
      }
      const auto middle = first + left_size;
      photons_[pos] = *middle;
      photons_[pos].axis = axis;
      buildPhotonMap(first, middle, pos * 2 + 1);
      buildPhotonMap(middle + 1, last, pos * 2 + 2);
    }

    std::vector<Photon> kNearestNeighbours(const vector3_type& point,
                                           real_type radius,
                                           size_t k) const {
      std::vector<Photon> heap;
      heap.reserve(k + 1);
      kNearestNeighbours(heap, 0, point, radius * radius, k);
      std::sort_heap(heap.begin(), heap.end(), PhotonComparator(point));
      return heap;
    }

    void kNearestNeighbours(std::vector<Photon>& heap,
                            size_t pos,
                            const vector3_type& point,
                            real_type squared_radius,
                            size_t k) const {
      if (pos >= photons_.size()) {
        return;
      }
      const auto& photon = photons_[pos];
      real_type plane_distance;
      switch (photon.axis) {
      case Axis::x:
        plane_distance = point.x() - photon.position.x();
        break;
      case Axis::y:
        plane_distance = point.y() - photon.position.y();
        break;
      case Axis::z:
        plane_distance = point.z() - photon.position.z();
        break;
      }
      size_t near, far;
      if (plane_distance < 0) {
        near = pos * 2 + 1;
        far = pos * 2 + 2;
      } else {
        near = pos * 2 + 2;
        far = pos * 2 + 1;
      }
      kNearestNeighbours(heap, near, point, squared_radius, k);
      if (heap.size() >= k) {
        squared_radius = (heap.front().position - point).squaredLength();
      }
      if (plane_distance * plane_distance < squared_radius) {
        kNearestNeighbours(heap, far, point, squared_radius, k);
      }
      if ((photon.position - point).squaredLength() < squared_radius) {
        heap.push_back(photon);
        std::push_heap(heap.begin(), heap.end(), PhotonComparator(point));
      }
      while (heap.size() > k) {
        std::pop_heap(heap.begin(), heap.end(), PhotonComparator(point));
        heap.pop_back();
      }
    }
  };

  size_t n_photon_, n_nearest_photon_;

public:
  PhotonMapping(size_t n_photon, size_t n_nearest_photon) :
    n_photon_(n_photon), n_nearest_photon_(n_nearest_photon) {}

  std::string to_string() const {
    std::stringstream ss;
    ss << "RealType: " << sizeof(real_type) * 8 << "bit" << std::endl;
    ss << "Shader: PhotonMapping(n_photon=" << n_photon_ << ", n_nearest_photon=" << n_nearest_photon_ << ")";
    return ss.str();
  }

  progress_const_reference
  render(const object_buffer_type& objects, const camera_type& camera) const {
    Random random;
    const acceleration_type acceleration(objects);
    const light_set_type lights(objects);

    std::cerr << "(1/3) Photon tracing ... ";
    std::vector<Photon> photons;
    for (size_t i = 0; i < n_photon_; i++) {
      const auto samples = photonTracing(acceleration, lights, random);
      std::move(samples.begin(), samples.end(), std::back_inserter(photons));
    }
    for (auto& photon : photons) {
      photon.power /= n_photon_;
    }
    std::cerr << "done." << std::endl;
    std::cerr << "      " << photons.size() << " photons are sampled." << std::endl;

    std::cerr << "(2/3) Building photon maps ... ";
    const PhotonMap photon_map(std::move(photons));
    std::cerr << "done." << std::endl;

    std::cerr << "(3/3) Rendering ... " << std::endl;
    for (size_t y = 0; y < camera.image_height(); y++) {
      for (size_t x = 0; x < camera.image_width(); x++) {
        std::cerr << "\ry = " << y << std::flush;
        radiant_type power;
        power += rendering(acceleration, camera, photon_map, x, y, random);
        camera.expose(x, y, power);
      }
    }
    std::cerr << "done." << std::endl;

    // TODO
    auto progress = std::make_shared<progress_type>(1, std::vector<std::thread>());
    progress->done(1);
    using namespace std::literals;
    std::this_thread::sleep_for(1ms);
    progress->end();
    return progress;
  }

private:
  std::vector<Photon>
  photonTracing(const acceleration_type& acceleration,
                const light_set_type& lights,
                Random& random) const {
    const auto light = lights.sample(random);
    const auto sample = light.sample_initial_ray(random);

    auto power = lights.total_power();
    auto ray = sample.ray;
    hit_type hit;
    object_type object;

    std::vector<Photon> photons;

    for (;;) {
      std::tie(hit, object) = acceleration.cast(ray);
      if (!hit) {
        break;
      }

      if (object.surface_type() == material::SurfaceType::diffuse) {
        Photon photon;
        photon.position = hit.position;
        photon.direction = -ray.direction;
        photon.power = power;
        photons.push_back(photon);
      }

      const auto sample =
        object.sample_scattering(power, -ray.direction, hit.normal, random);
      ray = ray_type(hit.position, sample.direction_o);
      const auto reflectance = sample.bsdf / sample.psa_probability;
      power *= reflectance;

      if (object.surface_type() == material::SurfaceType::specular) {
        continue;
      }

      const auto p_russian_roulette = reflectance.max();
      if (random.uniform<real_type>() >= p_russian_roulette) {
        break;
      }
      power /= std::min<real_type>(1, p_russian_roulette);
    }

    return photons;
  }

  radiant_type
  rendering(const acceleration_type& acceleration,
            const camera_type& camera,
            const PhotonMap& photon_map,
            size_t x, size_t y,
            Random& random) const {
    radiant_type power, weight(1);
    auto ray = camera.sample_initial_ray(x, y, random).ray;
    hit_type hit;
    object_type object;

    for (;;) {
      std::tie(hit, object) = acceleration.cast(ray);
      if (!hit) {
        break;
      }

      if (object.is_emissive() && dot(hit.normal, ray.direction) < 0) {
        power += object.emittance();
      }

      if (object.surface_type() == material::SurfaceType::diffuse) {
        const auto photons =
          photon_map.kNearestNeighbours(hit.position, 1, n_nearest_photon_);
        power += weight * gaussianFilter(photons, hit, object, -ray.direction);
        break;
      }

      const auto sample =
        object.sample_scattering(weight, -ray.direction, hit.normal, random);
      ray = ray_type(hit.position, sample.direction_o);
      const auto reflectance = sample.bsdf / sample.psa_probability;
      weight *= reflectance;
    }

    return power;
  }

  radiant_type
  gaussianFilter(const std::vector<Photon>& photons,
                 const hit_type& hit,
                 const object_type& object,
                 const vector3_type& direction_o) const {
    const real_type alpha = 0.918;
    const real_type beta = 1.953;
    const auto squared_max_distance =
      (photons.back().position - hit.position).squaredLength();
    radiant_type power;
    for (const auto& photon : photons) {
      const auto squared_distance =
        (photon.position - hit.position).squaredLength();
      const auto weight = 1 -
        (1 - std::exp(-beta * squared_distance / 2 / squared_max_distance)) /
        (1 - std::exp(-beta));
      const auto bsdf = object.bsdf(photon.direction, direction_o, hit.normal);
      power += bsdf * photon.power * weight;
    }
    return power * alpha / static_cast<real_type>(kPI) / squared_max_distance;
  }
};

}
}
