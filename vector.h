#pragma once

#include <algorithm>
#include <cmath>
#include <ostream>
#include <tuple>

namespace amber {

template <typename RealType>
struct Vector3
{
  using real_type    = RealType;

  using vector3_type = Vector3<real_type>;

  real_type x, y, z;

  explicit Vector3() :
    x(real_type()), y(real_type()), z(real_type())
  {}

  explicit Vector3(real_type k) :
    x(k), y(k), z(k)
  {}

  Vector3(real_type x, real_type y, real_type z) :
    x(x), y(y), z(z)
  {}

  real_type& operator[](size_t pos) noexcept
  {
    switch (pos) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("invalid position");
    }
  }

  const real_type& operator[](size_t pos) const noexcept
  {
    switch (pos) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
    default:
      throw std::runtime_error("invalid position");
    }
  }

  vector3_type& operator+=(const vector3_type& a) noexcept
  {
    *this = *this + a;
    return *this;
  }

  vector3_type& operator-=(const vector3_type& a) noexcept
  {
    *this = *this - a;
    return *this;
  }

  vector3_type& operator*=(const vector3_type& a) noexcept
  {
    *this = *this * a;
    return *this;
  }

  vector3_type& operator*=(real_type a) noexcept
  {
    *this = *this * a;
    return *this;
  }

  vector3_type& operator/=(const vector3_type& a) noexcept
  {
    *this = *this / a;
    return *this;
  }

  vector3_type& operator/=(real_type a) noexcept
  {
    *this = *this / a;
    return *this;
  }
};

template <typename RealType>
std::ostream& operator<<(std::ostream& os, const Vector3<RealType>& a)
{
  os << "(" << a.x << " " << a.y << " " << a.z << ")";
  return os;
}

template <typename RealType>
Vector3<RealType>& operator+(const Vector3<RealType>& a) noexcept
{
  return a;
}

template <typename RealType>
Vector3<RealType> operator-(const Vector3<RealType>& a) noexcept
{
  return Vector3<RealType>(-a.x, -a.y, -a.z);
}

template <typename RealType>
Vector3<RealType> operator+(const Vector3<RealType>& a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(a.x + b.x, a.y + b.y, a.z + b.z);
}

template <typename RealType>
Vector3<RealType> operator+(const Vector3<RealType>& a, RealType b) noexcept
{
  return Vector3<RealType>(a.x + b, a.y + b, a.z + b);
}

template <typename RealType>
Vector3<RealType> operator+(RealType a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(a + b.x, a + b.y, a + b.z);
}

template <typename RealType>
Vector3<RealType> operator-(const Vector3<RealType>& a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(a.x - b.x, a.y - b.y, a.z - b.z);
}

template <typename RealType>
Vector3<RealType> operator-(const Vector3<RealType>& a, RealType b) noexcept
{
  return Vector3<RealType>(a.x - b, a.y - b, a.z - b);
}

template <typename RealType>
Vector3<RealType> operator-(RealType a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(a - b.x, a - b.y, a - b.z);
}

template <typename RealType>
Vector3<RealType> operator*(const Vector3<RealType>& a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(a.x * b.x, a.y * b.y, a.z * b.z);
}

template <typename RealType>
Vector3<RealType> operator*(const Vector3<RealType>& a, RealType k) noexcept
{
  return Vector3<RealType>(a.x * k, a.y * k, a.z * k);
}

template <typename RealType>
Vector3<RealType> operator*(RealType k, const Vector3<RealType>& a) noexcept
{
  return Vector3<RealType>(k * a.x, k * a.y, k * a.z);
}

template <typename RealType>
Vector3<RealType> operator/(const Vector3<RealType>& a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(a.x / b.x, a.y / b.y, a.z / b.z);
}

template <typename RealType>
Vector3<RealType> operator/(const Vector3<RealType>& a, RealType k) noexcept
{
  return Vector3<RealType>(a.x / k, a.y / k, a.z / k);
}

template <typename RealType>
Vector3<RealType> operator/(RealType k, const Vector3<RealType>& a) noexcept
{
  return Vector3<RealType>(k / a.x, k / a.y, k / a.z);
}

template <typename RealType>
RealType max(const Vector3<RealType>& a) noexcept
{
  return std::max({a.x, a.y, a.z});
}

template <typename RealType>
RealType min(const Vector3<RealType>& a) noexcept
{
  return std::min({a.x, a.y, a.z});
}

template <typename RealType>
RealType dot(const Vector3<RealType>& a, const Vector3<RealType>& b) noexcept
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

template <typename RealType>
RealType norm2(const Vector3<RealType>& a) noexcept
{
  return dot(a, a);
}

template <typename RealType>
RealType norm(const Vector3<RealType>& a) noexcept
{
  return std::sqrt(norm2(a));
}

template <typename RealType>
Vector3<RealType> normalize(const Vector3<RealType>& a) noexcept
{
  return a / norm(a);
}

template <typename RealType>
Vector3<RealType> cross(const Vector3<RealType>& a, const Vector3<RealType>& b) noexcept
{
  return Vector3<RealType>(
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x
  );
}

template <typename RealType>
std::tuple<Vector3<RealType>, Vector3<RealType>> orthonormal_basis(const Vector3<RealType>& w) noexcept
{
  const auto u = normalize(cross(w, std::abs(w.x) < std::abs(w.y) ? Vector3<RealType>(1, 0, 0) : Vector3<RealType>(0, 1, 0)));
  const auto v = normalize(cross(w, u));
  return std::make_tuple(u, v);
}

}
