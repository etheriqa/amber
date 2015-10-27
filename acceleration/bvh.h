/*
 * Copyright (c) 2015 Kaede Takamori <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <limits>
#include <unordered_set>
#include <vector>

#include "acceleration/acceleration.h"

namespace amber {
namespace acceleration {

template <typename Object, size_t TraverseCost = 500, size_t IntersectionCost = 100>
class BVH : public Acceleration<Object>
{
public:
  using acceleration_type  = Acceleration<Object>;

  using hit_type           = typename acceleration_type::hit_type;
  using object_type        = typename acceleration_type::object_type;
  using ray_type           = typename acceleration_type::ray_type;
  using real_type          = typename acceleration_type::object_type::real_type;

  using object_buffer_type = std::vector<Object>;

private:
  enum struct Axis
  {
    None = -1,
    X    = 0,
    Y    = 1,
    Z    = 2,
  };

  enum struct EventType
  {
    End    = 0,
    Planar = 1,
    Start  = 2,
  };

  struct Event
  {
    EventType type;
    Axis axis;
    real_type position;
    object_type object;

    Event(EventType type, Axis axis, real_type position, const object_type& object) :
      type(type),
      axis(axis),
      position(position),
      object(object)
    {}

    bool operator<(const Event& event) const noexcept
    {
      return axis < event.axis ||
        (axis == event.axis && (position < event.position ||
        (position == event.position && type < event.type)));
    }
  };

  struct Node
  {
    using aabb_type       = typename object_type::aabb_type;
    using event_list_type = std::vector<Event>;


    Node *m_left, *m_right;
    object_buffer_type *m_objects;
    aabb_type m_voxel;

    Node(const object_buffer_type& objects) :
      Node(objects.size(), build_event_list(objects), aabb(objects))
    {}

    Node(size_t n, event_list_type events, const aabb_type& voxel) :
      m_left(nullptr),
      m_right(nullptr),
      m_objects(nullptr),
      m_voxel(voxel)
    {
      size_t n_left, n_right;
      event_list_type left_events, right_events;
      aabb_type left_voxel, right_voxel;
      std::tie(n_left, n_right, left_events, right_events, left_voxel, right_voxel) = split(n, events, voxel);

      if (n_left == n) {
        m_objects = new object_buffer_type;
        for (const auto& event : left_events) {
          if (event.axis == Axis::X && event.type != EventType::End) {
            m_objects->push_back(event.object);
          }
        }
        return;
      }

      events.clear();

      m_left = new Node(n_left, std::move(left_events), left_voxel);
      m_right = new Node(n_right, std::move(right_events), right_voxel);
    }

    ~Node()
    {
      delete m_left;
      delete m_right;
      delete m_objects;
    }

    std::tuple<hit_type, object_type> cast(const ray_type& ray, real_type t_max) const noexcept
    {
      if (m_objects != nullptr) {
        return acceleration_type::traverse(m_objects->begin(), m_objects->end(), ray, t_max);
      }

      bool left_hit, right_hit;
      real_type t_left, t_right;
      std::tie(left_hit, t_left, std::ignore) = m_left->m_voxel.intersect(ray, t_max);
      std::tie(right_hit, t_right, std::ignore) = m_right->m_voxel.intersect(ray, t_max);

      if (!left_hit && !right_hit) {
        return std::make_tuple(hit_type(), object_type());
      } else if (left_hit && !right_hit) {
        return m_left->cast(ray, t_max);
      } else if (!left_hit && right_hit) {
        return m_right->cast(ray, t_max);
      }

      const auto& near = t_left < t_right ? m_left : m_right;
      const auto& far = t_left < t_right ? m_right : m_left;

      hit_type near_hit;
      object_type near_object;
      std::tie(near_hit, near_object) = near->cast(ray, t_max);
      if (!near_hit) {
        return far->cast(ray, t_max);
      } else if (near_hit.distance < std::max(t_left, t_right)) {
        return std::make_tuple(near_hit, near_object);
      }

      hit_type far_hit;
      object_type far_object;
      std::tie(far_hit, far_object) = far->cast(ray, near_hit.distance);
      if (far_hit) {
        return std::make_tuple(far_hit, far_object);
      } else {
        return std::make_tuple(near_hit, near_object);
      }
    }

    static aabb_type aabb(const object_buffer_type& objects) noexcept
    {
      aabb_type aabb;
      for (const auto& object : objects) {
        aabb += object.aabb();
      }
      return aabb;
    }

    static event_list_type build_event_list(const object_buffer_type& objects) noexcept
    {
      event_list_type events;
      for (const auto& object : objects) {
        const auto voxel = object.aabb();
        for (const auto axis : {Axis::X, Axis::Y, Axis::Z}) {
          const auto& start = voxel.min[static_cast<size_t>(axis)];
          const auto& end = voxel.max[static_cast<size_t>(axis)];
          if (start == end) {
            events.emplace_back(EventType::Planar, axis, start, object);
          } else {
            events.emplace_back(EventType::End, axis, end, object);
            events.emplace_back(EventType::Start, axis, start, object);
          }
        }
      }

      std::sort(events.begin(), events.end());

      return events;
    }

    static std::tuple<size_t, size_t, event_list_type, event_list_type, aabb_type, aabb_type> split(size_t n, const event_list_type& events, const aabb_type& voxel) noexcept
    {
      const auto surface_area = voxel.surfaceArea();

      real_type optimal_cost = IntersectionCost * n;
      Axis optimal_axis = Axis::None;
      bool optimal_left_including;
      size_t optimal_n_left;

      for (const auto& axis : {Axis::X, Axis::Y, Axis::Z}) {
        std::vector<aabb_type> left_including_voxels(1), left_intersection_voxels(1);
        for (auto it = events.begin(); it != events.end(); it++) {
          const auto& event = *it;
          const auto object_voxel = event.object.aabb();
          if (event.type == EventType::End || event.type == EventType::Planar) {
            left_including_voxels.push_back(left_including_voxels.back() + object_voxel);
          }
          if (event.type == EventType::Planar || event.type == EventType::Start) {
            left_intersection_voxels.push_back(left_intersection_voxels.back() + object_voxel);
          }
        }
        std::vector<aabb_type> right_including_voxels(1), right_intersection_voxels(1);
        for (auto it = events.rbegin(); it != events.rend(); it++) {
          const auto& event = *it;
          const auto object_voxel = event.object.aabb();
          if (event.type == EventType::End || event.type == EventType::Planar) {
            right_intersection_voxels.push_back(right_intersection_voxels.back() + object_voxel);
          }
          if (event.type == EventType::Planar || event.type == EventType::Start) {
            right_including_voxels.push_back(right_including_voxels.back() + object_voxel);
          }
        }
        for (size_t i = 1; i < n; i++) {
          const auto cost = surface_area_heuristic(
            surface_area,
            left_including_voxels[i].surfaceArea(),
            right_intersection_voxels[n - i].surfaceArea(),
            i,
            n - i
          );
          if (cost < optimal_cost) {
            optimal_cost = cost;
            optimal_axis = axis;
            optimal_left_including = true;
            optimal_n_left = i;
          }
        }
        for (size_t i = 1; i < n; i++) {
          const auto cost = surface_area_heuristic(
            surface_area,
            left_intersection_voxels[i].surfaceArea(),
            right_including_voxels[n - i].surfaceArea(),
            i,
            n - i
          );
          if (cost < optimal_cost) {
            optimal_cost = cost;
            optimal_axis = axis;
            optimal_left_including = false;
            optimal_n_left = i;
          }
        }
      }

      if (optimal_axis == Axis::None) {
        return std::make_tuple(n, 0, events, event_list_type(), voxel, aabb_type());
      }

      std::unordered_set<object_type, typename object_type::Hash, typename object_type::EqualTo> left_objects;
      aabb_type left_voxel, right_voxel;
      size_t i = 0;
      for (const auto& event : events) {
        if (event.axis != optimal_axis) {
          continue;
        }
        if (optimal_left_including && event.type == EventType::Start) {
          continue;
        }
        if (!optimal_left_including && event.type == EventType::End) {
          continue;
        }
        if (i < optimal_n_left) {
          left_objects.insert(event.object);
          left_voxel += event.object.aabb();
        } else {
          right_voxel += event.object.aabb();
        }
        i++;
      }

      event_list_type left_events, right_events;
      for (const auto& event : events) {
        (left_objects.count(event.object) == 1 ? left_events : right_events).push_back(event);
      }

      return std::make_tuple(optimal_n_left, n - optimal_n_left, left_events, right_events, left_voxel, right_voxel);
    }

    static real_type surface_area_heuristic(real_type surface_area, real_type left_surface_area, real_type right_surface_area, size_t n_left, size_t n_right) noexcept
    {
      return TraverseCost + IntersectionCost * (left_surface_area * n_left + right_surface_area * n_right) / surface_area;
    }
  };

  Node *m_root;

public:
  explicit BVH(const object_buffer_type& objects) : m_root(new Node(objects)) {}

  ~BVH()
  {
    delete m_root;
  }

  void write(std::ostream& os) const noexcept {
    os
      << "BVH(traverse_cost=" << TraverseCost
      << ", intersection_cost=" << IntersectionCost
      << ")";
  }

  std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
  {
    return m_root->cast(ray, std::numeric_limits<real_type>::max());
  }
};

}
}
