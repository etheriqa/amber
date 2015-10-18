#pragma once

#include <algorithm>
#include <future>
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

  enum struct PhotonType {
    caustic,
    global,
  };

  struct Photon {
    vector3_type position;
    vector3_type direction_i;
    radiant_type power_i;
  };

  struct PhotonCompare {
    vector3_type point;
    explicit PhotonCompare(const vector3_type& point) : point(point) {}
    bool operator()(const Photon& a, const Photon& b) const {
      return norm2(a.position - point) < norm2(a.position - point);
    }
  };

  struct PhotonMapNode {
    Axis axis;
    Photon photon;
    PhotonMapNode *left = nullptr, *right = nullptr;

    template <typename RandomAccessIterator>
    PhotonMapNode(RandomAccessIterator first, RandomAccessIterator last) {
      aabb_type voxel;
      std::for_each(first, last, [&](const auto& photon){
        voxel += aabb_type(photon.position);
      });
      const auto x = voxel.max.x - voxel.min.x;
      const auto y = voxel.max.y - voxel.min.y;
      const auto z = voxel.max.z - voxel.min.z;
      if (x > y && x > z) {
        axis = Axis::x;
      } else if (y > z) {
        axis = Axis::y;
      } else {
        axis = Axis::z;
      }
      // TODO use std::partial_sort
      std::sort(first, last, [&](const auto& a, const auto& b){
        switch (axis) {
        case Axis::x:
          return a.position.x < b.position.x;
        case Axis::y:
          return a.position.y < b.position.y;
        case Axis::z:
          return a.position.z < b.position.z;
        }
      });
      const auto median = first + std::distance(first, last) / 2;
      photon = *median;
      if (std::distance(first, median) > 0) {
        left = new PhotonMapNode(first, median);
      }
      if (std::distance(median + 1, last) > 0) {
        right = new PhotonMapNode(median + 1, last);
      }
    }

    PhotonMapNode(const PhotonMapNode&) = delete;

    ~PhotonMapNode() {
      delete left;
      delete right;
    }

    PhotonMapNode& operator=(const PhotonMapNode&) = delete;

    std::vector<Photon>
    searchNearestNeighbourPhotons(const vector3_type& point,
                                  real_type squared_radius,
                                  size_t n) const {
      real_type plane_distance;
      switch (axis) {
      case Axis::x:
        plane_distance = point.x - photon.position.x;
        break;
      case Axis::y:
        plane_distance = point.y - photon.position.y;
        break;
      case Axis::z:
        plane_distance = point.z - photon.position.z;
        break;
      }
      const auto squared_plane_distance = plane_distance * plane_distance;

      std::vector<Photon> left_photons, right_photons;
      if (plane_distance < 0) {
        if (left != nullptr) {
          left_photons =
            left->searchNearestNeighbourPhotons(point, squared_radius, n);
          if (left_photons.size() == n) {
            squared_radius =
              std::min(squared_radius,
                       norm2(left_photons.back().position - point));
          }
        }
        if (right != nullptr && squared_plane_distance < squared_radius) {
          right_photons =
            right->searchNearestNeighbourPhotons(point, squared_radius, n);
        }
      } else {
        if (right != nullptr) {
          right_photons =
            right->searchNearestNeighbourPhotons(point, squared_radius, n);
          if (right_photons.size() == n) {
            squared_radius =
              std::min(squared_radius,
                       norm2(right_photons.back().position - point));
          }
        }
        if (left != nullptr && squared_plane_distance < squared_radius) {
          left_photons =
            left->searchNearestNeighbourPhotons(point, squared_radius, n);
        }
      }

      std::vector<Photon> photons;
      std::move(left_photons.begin(), left_photons.end(),
                std::back_inserter(photons));
      std::move(right_photons.begin(), right_photons.end(),
                std::back_inserter(photons));
      photons.push_back(photon);
      std::inplace_merge(photons.begin(),
                         photons.begin() + left_photons.size(),
                         photons.begin() + left_photons.size() + right_photons.size(),
                         [&](const auto& a, const auto& b){
        return norm2(a.position - point) < norm2(b.position - point);
      });
      std::inplace_merge(photons.begin(),
                         photons.begin() + left_photons.size() + right_photons.size(),
                         photons.end(),
                         [&](const auto& a, const auto& b){
        return norm2(a.position - point) < norm2(b.position - point);
      });
      photons.resize(std::min(photons.size(), n));
      return photons;
    }
  };

  size_t n_thread, n_photon;

public:
  PhotonMapping(size_t n_thread, size_t n_photon) :
    n_thread(n_thread), n_photon(n_photon) {}

  std::string to_string() const {
    std::stringstream ss;
    ss << "RealType: " << sizeof(real_type) * 8 << "bit" << std::endl;
    ss << "PhotonMapping(n_photon=" << n_photon << ")";
    return ss.str();
  }

  progress_const_reference
  render(const object_buffer_type& objects, const camera_type& camera) const {
    Random random;
    const acceleration_type acceleration(objects);
    const light_set_type lights(objects);

    std::cerr << "(1/3) Photon tracing ... ";
    std::vector<Photon> caustic_photons, global_photons;
    for (size_t i = 0; i < n_photon; i++) {
      PhotonType type;
      std::vector<Photon> photons;
      std::tie(type, photons) = photonTracing(acceleration, lights, random);
      for (auto& photon : photons) {
        photon.power_i /= n_photon;
      }
      switch (type) {
      case PhotonType::caustic:
        std::move(photons.begin(), photons.end(), std::back_inserter(caustic_photons));
        break;
      case PhotonType::global:
        std::move(photons.begin(), photons.end(), std::back_inserter(global_photons));
        break;
      }
    }
    std::cerr << "done." << std::endl;
    std::cerr << "      " << caustic_photons.size() << " caustic photons are sampled." << std::endl;
    std::cerr << "      " << global_photons.size() << " global photons are sampled." << std::endl;

    std::cerr << "(2/3) Building photon maps ... ";
    const PhotonMapNode caustic_photon_map(caustic_photons.begin(),
                                           caustic_photons.end());
    const PhotonMapNode global_photon_map(global_photons.begin(),
                                          global_photons.end());
    std::cerr << "done." << std::endl;

    std::cerr << "(3/3) Rendering ... " << std::endl;
    for (size_t y = 0; y < camera.image_height(); y++) {
      std::cerr << "\ry = " << y;
      for (size_t x = 0; x < camera.image_width(); x++) {
        radiant_type power;
        power += rendering(acceleration, camera, caustic_photon_map, x, y, random);
        power += rendering(acceleration, camera, global_photon_map, x, y, random);
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
  std::tuple<PhotonType, std::vector<Photon>>
  photonTracing(const acceleration_type& acceleration,
                const light_set_type& lights,
                Random& random) const {
    const auto light = lights.sample(random);
    const auto sample = light.sample_initial_ray(random);
    auto power = light.emittance();

    auto ray = sample.ray;
    hit_type hit;
    object_type object;

    auto type = PhotonType::global;
    std::vector<Photon> photons;

    for (;;) {
      std::tie(hit, object) = acceleration.cast(ray);
      if (!hit) {
        break;
      }
      switch (object.surface_type()) {
      case material::SurfaceType::specular:
        if (photons.empty()) {
          type = PhotonType::caustic;
        }
        break;
      case material::SurfaceType::diffuse:
        Photon photon;
        photon.position = hit.position;
        photon.direction_i = -ray.direction;
        photon.power_i = power;
        photons.push_back(photon);
        break;
      }
      if (type == PhotonType::caustic && !photons.empty()) {
        break;
      }

      const auto sample = object.sample_scattering(-ray.direction, hit.normal, random);
      power *= sample.bsdf / sample.psa_probability;
      const auto p_russian_roulette = max(power);
      if (random.uniform<real_type>() >= p_russian_roulette) {
        break;
      }

      ray = ray_type(hit.position, sample.direction_o);
      power /= std::min(static_cast<real_type>(1), p_russian_roulette);
    }

    return std::make_tuple(type, photons);
  }

  radiant_type
  rendering(const acceleration_type& acceleration,
            const camera_type& camera,
            const PhotonMapNode& photon_map,
            size_t x, size_t y,
            Random& random) const {
    auto ray = camera.sample_initial_ray(x, y, random).ray;
    hit_type hit;
    object_type object;

    for (;;) {
      std::tie(hit, object) = acceleration.cast(ray);
      if (!hit) {
        return radiant_type();
      }
      // TODO
      if (object.surface_type() == material::SurfaceType::specular) {
        return radiant_type();
      }
      const auto photons =
        photon_map.searchNearestNeighbourPhotons(hit.position, 1, 10);
      if (photons.empty()) {
        return radiant_type();
      }
      const auto area = static_cast<real_type>(kPI) *
        norm2(photons.back().position - hit.position);
      radiant_type power;
      for (const auto& photon : photons) {
        power += photon.power_i *
          object.bsdf(photon.direction_i, -ray.direction, hit.normal);
      }
      return power / area;
    }
  }
};

}
}
