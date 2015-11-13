/*
 * Copyright (c) 2015 TAKAMORI Kaede <etheriqa@gmail.com>
 *
 * This file is part of the amber (https://github.com/etheriqa/amber).
 *
 * This software is released under the MIT License (the "LICENSE").
 */

#pragma once

#include <algorithm>
#include <iterator>
#include <limits>
#include <unordered_set>
#include <vector>

#include "acceleration.h"

namespace amber {
namespace acceleration {

template <
  typename Object,
  size_t TraverseCost = 500,
  size_t IntersectionCost = 100
>
class BVH : public Acceleration<Object>
{
public:
  using object_type = Object;

  using hit_type    = typename Object::hit_type;
  using ray_type    = typename Object::ray_type;

private:
  using real_type   = typename Object::real_type;

  enum struct Axis {
    None = -1,
    X    = 0,
    Y    = 1,
    Z    = 2,
  };

  enum struct EventType {
    End    = 0,
    Planar = 1,
    Start  = 2,
  };

  struct Event {
    EventType type;
    Axis axis;
    real_type position;
    Object object;

    Event(EventType type,
          Axis axis,
          real_type position,
          Object const& object) noexcept
      : type(type), axis(axis), position(position), object(object) {}

    bool operator<(Event const& event) const noexcept {
      return axis < event.axis ||
        (axis == event.axis && (position < event.position ||
        (position == event.position && type < event.type)));
    }
  };

  struct Node {
    using aabb_type        = typename Object::aabb_type;
    using event_list_type  = std::vector<Event>;
    using object_list_type = std::vector<Object>;

    Node *m_left, *m_right;
    object_list_type *m_objects;
    aabb_type m_voxel;

    template <typename InputIterator>
    Node(InputIterator first, InputIterator last)
      : Node(std::distance(first, last),
             aabb(first, last),
             build_event_list(first, last)) {}

    Node(size_t n, aabb_type const& voxel, event_list_type events)
      : m_left(nullptr), m_right(nullptr), m_objects(nullptr), m_voxel(voxel) {
      size_t n_left, n_right;
      event_list_type left_events, right_events;
      aabb_type left_voxel, right_voxel;
      std::tie(n_left, n_right, left_events, right_events, left_voxel, right_voxel) = split(n, events, voxel);

      if (n_left == n) {
        m_objects = new object_list_type;
        for (auto const& event : left_events) {
          if (event.axis == Axis::X && event.type != EventType::End) {
            m_objects->push_back(event.object);
          }
        }
        return;
      }

      events.clear();

      m_left = new Node(n_left, left_voxel, std::move(left_events));
      m_right = new Node(n_right, right_voxel, std::move(right_events));
    }

    ~Node() {
      delete m_left;
      delete m_right;
      delete m_objects;
    }

    std::tuple<hit_type, Object>
    cast(ray_type const& ray, real_type t_max) const noexcept {
      if (m_objects != nullptr) {
        return Acceleration<Object>::Traverse(m_objects->begin(),
                                              m_objects->end(),
                                              ray,
                                              t_max);
      }

      bool left_hit, right_hit;
      real_type t_left, t_right;
      std::tie(left_hit, t_left, std::ignore) =
        m_left->m_voxel.Intersect(ray, t_max);
      std::tie(right_hit, t_right, std::ignore) =
        m_right->m_voxel.Intersect(ray, t_max);

      if (!left_hit && !right_hit) {
        return std::make_tuple(hit_type(), Object());
      } else if (left_hit && !right_hit) {
        return m_left->cast(ray, t_max);
      } else if (!left_hit && right_hit) {
        return m_right->cast(ray, t_max);
      }

      auto const& near = t_left < t_right ? m_left : m_right;
      auto const& far = t_left < t_right ? m_right : m_left;

      hit_type near_hit;
      Object near_object;
      std::tie(near_hit, near_object) = near->cast(ray, t_max);
      if (!near_hit) {
        return far->cast(ray, t_max);
      } else if (near_hit.distance < std::max(t_left, t_right)) {
        return std::make_tuple(near_hit, near_object);
      }

      hit_type far_hit;
      Object far_object;
      std::tie(far_hit, far_object) = far->cast(ray, near_hit.distance);
      if (far_hit) {
        return std::make_tuple(far_hit, far_object);
      } else {
        return std::make_tuple(near_hit, near_object);
      }
    }

    template <typename InputIterator>
    static aabb_type
    aabb(InputIterator first, InputIterator last) noexcept {
      aabb_type aabb;
      std::for_each(first, last, [&](auto const& object){
        aabb += object.BoundingBox();
      });
      return aabb;
    }

    template <typename InputIterator>
    static event_list_type
    build_event_list(InputIterator first, InputIterator last) noexcept {
      event_list_type events;
      std::for_each(first, last, [&](auto const& object){
        auto const voxel = object.BoundingBox();
        for (auto const axis : {Axis::X, Axis::Y, Axis::Z}) {
          auto const& start = voxel.min[static_cast<size_t>(axis)];
          auto const& end = voxel.max[static_cast<size_t>(axis)];
          if (start == end) {
            events.emplace_back(EventType::Planar, axis, start, object);
          } else {
            events.emplace_back(EventType::End, axis, end, object);
            events.emplace_back(EventType::Start, axis, start, object);
          }
        }
      });
      std::sort(events.begin(), events.end());
      return events;
    }

    static std::tuple<size_t, size_t,
                      event_list_type, event_list_type,
                      aabb_type, aabb_type>
    split(size_t n,
          event_list_type const& events, aabb_type const& voxel) noexcept {
      auto const surface_area = voxel.SurfaceArea();

      real_type optimal_cost = IntersectionCost * n;
      Axis optimal_axis = Axis::None;
      bool optimal_left_including;
      size_t optimal_n_left;

      for (auto const& axis : {Axis::X, Axis::Y, Axis::Z}) {
        std::vector<aabb_type> left_including_voxels(1),
                               left_intersection_voxels(1);
        for (auto it = events.begin(); it != events.end(); it++) {
          auto const& event = *it;
          auto const object_voxel = event.object.BoundingBox();
          if (event.type == EventType::End ||
              event.type == EventType::Planar) {
            left_including_voxels.push_back(
              left_including_voxels.back() + object_voxel);
          }
          if (event.type == EventType::Planar ||
              event.type == EventType::Start) {
            left_intersection_voxels.push_back(
              left_intersection_voxels.back() + object_voxel);
          }
        }
        std::vector<aabb_type> right_including_voxels(1),
                               right_intersection_voxels(1);
        for (auto it = events.rbegin(); it != events.rend(); it++) {
          auto const& event = *it;
          auto const object_voxel = event.object.BoundingBox();
          if (event.type == EventType::End ||
              event.type == EventType::Planar) {
            right_intersection_voxels.push_back(
              right_intersection_voxels.back() + object_voxel);
          }
          if (event.type == EventType::Planar ||
              event.type == EventType::Start) {
            right_including_voxels.push_back(
              right_including_voxels.back() + object_voxel);
          }
        }
        for (size_t i = 1; i < n; i++) {
          auto const cost = surface_area_heuristic(
            surface_area,
            left_including_voxels[i].SurfaceArea(),
            right_intersection_voxels[n - i].SurfaceArea(),
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
          auto const cost = surface_area_heuristic(
            surface_area,
            left_intersection_voxels[i].SurfaceArea(),
            right_including_voxels[n - i].SurfaceArea(),
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
        return std::make_tuple(n, 0,
                               events, event_list_type(),
                               voxel, aabb_type());
      }

      std::unordered_set<Object,
                         typename Object::Hash,
                         typename Object::EqualTo> left_objects;
      aabb_type left_voxel, right_voxel;
      size_t i = 0;
      for (auto const& event : events) {
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
          left_voxel += event.object.BoundingBox();
        } else {
          right_voxel += event.object.BoundingBox();
        }
        i++;
      }

      event_list_type left_events, right_events;
      for (auto const& event : events) {
        if (left_objects.count(event.object) == 1) {
          left_events.push_back(event);
        } else {
          right_events.push_back(event);
        }
      }

      return std::make_tuple(optimal_n_left, n - optimal_n_left,
                             left_events, right_events,
                             left_voxel, right_voxel);
    }

    static real_type
    surface_area_heuristic(real_type surface_area,
                           real_type left_surface_area,
                           real_type right_surface_area,
                           size_t n_left,
                           size_t n_right) noexcept {
      return
        TraverseCost +
        IntersectionCost *
        (left_surface_area * n_left +
         right_surface_area * n_right) / surface_area;
    }
  };

  Node root_;

public:
  template <typename InputIterator>
  BVH(InputIterator first, InputIterator last) : root_(first, last) {}

  void Write(std::ostream& os) const noexcept
  {
    os
      << "BVH(Traverse_cost=" << TraverseCost
      << ", intersection_cost=" << IntersectionCost
      << ")";
  }

  std::tuple<hit_type, Object>
  Cast(const ray_type& ray) const noexcept
  {
    return root_.cast(ray, std::numeric_limits<real_type>::max());
  }
};

}
}
