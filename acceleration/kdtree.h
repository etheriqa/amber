#pragma once

#include <sstream>
#include <tuple>
#include <unordered_map>
#include <vector>
#include "acceleration/acceleration.h"

namespace amber {
namespace acceleration {

template <typename Object, size_t TraverseCost = 100, size_t IntersectionCost = 100, size_t MaxDepth = 24, size_t SplitSize = 16>
class KDTree : public Acceleration<Object>
{
public:
  using acceleration_type  = Acceleration<Object>;

  using hit_type           = typename acceleration_type::hit_type;
  using object_type        = typename acceleration_type::object_type;
  using ray_type           = typename acceleration_type::ray_type;
  using real_type          = typename acceleration_type::object_type::real_type;

  using object_buffer_type = std::vector<Object>;

private:
  using aabb_type          = typename object_type::aabb_type;

  enum Axis {
    kAxisNone = -1,
    kAxisX    = 0,
    kAxisY    = 1,
    kAxisZ    = 2,
  };

  enum EventType {
    kEventNone   = -1,
    kEventEnd    = 0,
    kEventPlanar = 1,
    kEventStart  = 2,
  };

  enum Side {
    kSideNone  = -1,
    kSideLeft  = 0,
    kSideRight = 1,
    kSideBoth  = 2,
  };

  struct SplitPlane
  {
    Axis axis;
    real_type position;

    explicit SplitPlane() :
      axis(kAxisNone),
      position()
    {}

    SplitPlane(Axis axis, real_type position) :
      axis(axis),
      position(position)
    {}

    bool operator==(const SplitPlane& plane) const noexcept
    {
      return axis == plane.axis && position == plane.position;
    }

    bool operator<(const SplitPlane& plane) const noexcept
    {
      return axis < plane.axis ||
        (axis == plane.axis && position < plane.position);
    }
  };

  struct Event
  {
    SplitPlane plane;
    EventType type;
    object_type object;

    explicit Event() :
      plane(),
      type(),
      object()
    {}

    Event(const SplitPlane& plane, EventType type, const object_type& object) :
      plane(plane),
      type(type),
      object(object)
    {}

    bool operator<(const Event& event) const noexcept
    {
      return plane < event.plane ||
        (plane == event.plane && type < event.type);
    }
  };

  struct Node
  {
    using event_list_type            = std::vector<Event>;
    using object_classification_type = std::unordered_map<object_type, Side, typename object_type::Hash, typename object_type::EqualTo>;

    Node *m_left, *m_right;
    aabb_type m_voxel;
    SplitPlane m_plane;
    object_buffer_type *m_objects;

    static aabb_type aabb(const object_buffer_type& objects) noexcept
    {
      auto aabb = aabb_type::empty();
      for (const auto& object : objects) {
        aabb += object.aabb();
      }
      return aabb;
    }

    static event_list_type build_event_list(const object_buffer_type& objects) noexcept
    {
      event_list_type events;
      Event event;
      for (const auto& object : objects) {
        const auto voxel = object.aabb();
        for (const auto axis : {kAxisX, kAxisY, kAxisZ}) {
          const auto& start = voxel.min[axis];
          const auto& end = voxel.max[axis];
          if (start == end) {
            events.emplace_back(SplitPlane(axis, start), kEventPlanar, object);
          } else {
            events.emplace_back(SplitPlane(axis, end), kEventEnd, object);
            events.emplace_back(SplitPlane(axis, start), kEventStart, object);
          }
        }
      }

      std::sort(events.begin(), events.end());

      return events;
    }

    static std::tuple<SplitPlane, Side> find_plane(size_t n, const aabb_type& voxel, const event_list_type& events) noexcept
    {
      std::vector<size_t> n_left_set(3, 0);
      std::vector<size_t> n_planar_set(3, 0);
      std::vector<size_t> n_right_set(3, n);

      real_type optimal_cost = IntersectionCost * n;
      SplitPlane optimal_plane;
      Side optimal_side = kSideNone;

      for (size_t i = 0; i < events.size();) {
        const auto& plane = events[i].plane;
        auto& n_left = n_left_set[plane.axis];
        auto& n_planar = n_planar_set[plane.axis];
        auto& n_right = n_right_set[plane.axis];
        size_t n_end_event = 0;
        size_t n_planar_event = 0;
        size_t n_start_event = 0;
        while (i < events.size() && events[i].plane == plane) {
          switch (events[i].type) {
          case kEventNone:
            throw std::logic_error("");
            break;
          case kEventEnd:
            n_end_event++;
            break;
          case kEventPlanar:
            n_planar_event++;
            break;
          case kEventStart:
            n_start_event++;
            break;
          }
          i++;
        }

        n_planar = n_planar_event;
        n_right -= n_end_event + n_planar_event;

        real_type cost;
        Side side;
        std::tie(cost, side) = surface_area_heuristic(voxel, plane, n_left, n_right, n_planar);
        if (cost < optimal_cost) {
          optimal_cost = cost;
          optimal_plane = plane;
          optimal_side = side;
        }

        n_planar = 0;
        n_left += n_start_event + n_planar_event;
      }

      return std::make_tuple(optimal_plane, optimal_side);
    }

    static std::tuple<real_type, Side> surface_area_heuristic(const aabb_type& voxel, const SplitPlane& plane, size_t n_left, size_t n_right, size_t n_planar) noexcept
    {
      aabb_type left_voxel, right_voxel;
      std::tie(left_voxel, right_voxel) = split_box(voxel, plane);

      const auto surface_area = voxel.surface_area();
      const auto p_left = left_voxel.surface_area() / surface_area;
      const auto p_right = right_voxel.surface_area() / surface_area;
      const auto cost_left = cost(voxel, plane, p_left, p_right, n_left + n_planar, n_right);
      const auto cost_right = cost(voxel, plane, p_left, p_right, n_left, n_right + n_planar);

      if (voxel.min[plane.axis] == plane.position) {
        return std::make_tuple(cost_left, kSideLeft);
      }

      if (voxel.max[plane.axis] == plane.position) {
        return std::make_tuple(cost_right, kSideRight);
      }

      if (cost_left < cost_right) {
        return std::make_tuple(cost_left, kSideLeft);
      } else {
        return std::make_tuple(cost_right, kSideRight);
      }
    }

    static std::tuple<aabb_type, aabb_type> split_box(const aabb_type& voxel, const SplitPlane& plane) noexcept
    {
      auto left_voxel = voxel;
      auto right_voxel = voxel;
      left_voxel.max[plane.axis] = plane.position;
      right_voxel.min[plane.axis] = plane.position;
      return std::make_tuple(left_voxel, right_voxel);
    }

    static real_type cost(const aabb_type& voxel, const SplitPlane& plane, real_type p_left, real_type p_right, size_t n_left, size_t n_right) noexcept
    {
      return lambda(n_left, n_right) * (TraverseCost + IntersectionCost * (p_left * n_left + p_right * n_right));
    }

    static real_type lambda(size_t n_left, size_t n_right) noexcept
    {
      return n_left * n_right == 0 ? 0.8 : 1.0;
    }

    static object_classification_type classify_objects(const event_list_type& events, const SplitPlane& plane, Side side) noexcept
    {
      object_classification_type classification;

      for (const auto& event : events) {
        classification[event.object] = kSideBoth;
      }

      for (const auto& event : events) {
        if (event.plane.axis != plane.axis) {
          continue;
        }

        switch (event.type) {
        case kEventNone:
          throw std::logic_error("");
          break;
        case kEventEnd:
          if (event.plane.position < plane.position || (event.plane.position == plane.position && side == kSideLeft)) {
            classification[event.object] = kSideLeft;
          }
          break;
        case kEventPlanar:
          if (event.plane.position < plane.position || (event.plane.position == plane.position && side == kSideLeft)) {
            classification[event.object] = kSideLeft;
          }
          if (event.plane.position > plane.position || (event.plane.position == plane.position && side == kSideRight)) {
            classification[event.object] = kSideRight;
          }
          break;
        case kEventStart:
          if (event.plane.position > plane.position || (event.plane.position == plane.position && side == kSideRight)) {
            classification[event.object] = kSideRight;
          }
          break;
        }
      }

      return classification;
    }

    Node(const object_buffer_type& objects) :
      Node(objects.size(), aabb(objects), std::move(build_event_list(objects)), 0)
    {}

    Node(size_t n, const aabb_type& voxel, event_list_type&& events, size_t depth) :
      m_left(nullptr),
      m_right(nullptr),
      m_voxel(voxel),
      m_plane(),
      m_objects(nullptr)
    {
      Side side;
      std::tie(m_plane, side) = find_plane(n, m_voxel, events);
      const auto classification = classify_objects(events, m_plane, side);

      if (depth > MaxDepth || n <= SplitSize || m_plane.axis == kAxisNone) {
        m_objects = new object_buffer_type;
        for (const auto& pair : classification) {
          m_objects->push_back(pair.first);
        }
        return;
      }

      size_t n_left = 0;
      size_t n_right = 0;
      for (const auto& pair : classification) {
        const auto& side = pair.second;
        switch (side) {
        case kSideNone:
          throw std::logic_error("");
          break;
        case kSideLeft:
          n_left++;
          break;
        case kSideRight:
          n_right++;
          break;
        case kSideBoth:
          n_left++;
          n_right++;
          break;
        }
      }

      event_list_type left_events, right_events;
      for (const auto& event : events) {
        switch (classification.at(event.object)) {
        case kSideNone:
          throw std::logic_error("");
          break;
        case kSideLeft:
          left_events.push_back(event);
          break;
        case kSideRight:
          right_events.push_back(event);
          break;
        case kSideBoth:
          left_events.push_back(event);
          right_events.push_back(event);
          break;
        }
      }

      aabb_type left_voxel, right_voxel;
      std::tie(left_voxel, right_voxel) = split_box(m_voxel, m_plane);

      events.clear();
      if (n_left > 0) {
        m_left = new Node(n_left, left_voxel, std::move(left_events), depth + 1);
      }
      if (n_right > 0) {
        m_right = new Node(n_right, right_voxel, std::move(right_events), depth + 1);
      }
    }

    ~Node()
    {
      delete m_left;
      delete m_right;
      delete m_objects;
    }

    std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
    {
      if (m_objects != nullptr) {
        return acceleration_type::traverse(m_objects->begin(), m_objects->end(), ray);
      }

      const auto& ray_origin = ray.origin[m_plane.axis];
      const auto& ray_direction = ray.direction[m_plane.axis];

      if (ray_origin < m_plane.position && ray_direction <= 0) {
        return m_left == nullptr
          ? std::make_tuple(hit_type(), object_type())
          : m_left->cast(ray);
      }
      if (ray_origin > m_plane.position && ray_direction >= 0) {
        return m_right == nullptr
          ? std::make_tuple(hit_type(), object_type())
          : m_right->cast(ray);
      }
      if (m_right == nullptr) {
        return m_left->cast(ray);
      }
      if (m_left == nullptr) {
        return m_right->cast(ray);
      }

      bool left_hit, right_hit;
      real_type left_t, right_t;
      std::tie(left_hit, std::ignore, left_t) = m_left->m_voxel.intersect(ray);
      std::tie(right_hit, std::ignore, right_t) = m_right->m_voxel.intersect(ray);

      if (!left_hit && !right_hit) {
        return std::make_tuple(hit_type(), object_type());
      } else if (left_hit && !right_hit) {
        return m_left->cast(ray);
      } else if (!left_hit && right_hit) {
        return m_right->cast(ray);
      }

      hit_type hit;
      object_type object;

      const auto& near = left_t < right_t ? m_left : m_right;
      const auto& near_distance = left_t < right_t ? left_t : right_t;
      std::tie(hit, object) = near->cast(ray);
      if (hit && hit.distance <= near_distance) {
        return std::make_tuple(hit, object);
      }

      const auto& far = left_t < right_t ? m_right : m_left;
      const auto& far_distance = left_t < right_t ? right_t : left_t;
      std::tie(hit, object) = far->cast(ray);
      if (hit && hit.distance <= far_distance) {
        return std::make_tuple(hit, object);
      }

      return std::make_tuple(hit_type(), object_type());
    }
  };

  Node *m_root;

public:
  static std::string to_string() noexcept
  {
    std::stringstream ss;
    ss << "KDTree";
    return ss.str();
  }

  explicit KDTree(const object_buffer_type& objects) : m_root(new Node(objects)) {}

  ~KDTree()
  {
    delete m_root;
  }

  std::tuple<hit_type, object_type> cast(const ray_type& ray) const noexcept
  {
    return m_root->cast(ray);
  }
};

}
}
