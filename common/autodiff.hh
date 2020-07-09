#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>
#include <limits>

#define DBG_MACRO_NO_WARNING
#include <dbg.h>

namespace addn {
struct DN {
  double v;
  double a;

  DN(double _v, double _a) : v(_v), a(_a) {}
  DN(double _v) : DN(_v, 0.0) {}
  DN(const DN& x) : DN(x.v, x.a) {}
  DN() : DN(0.0, 0.0) {}
  inline DN& operator=(const DN& x) {
    v = x.v;
    a = x.a;
    return *this;
  }

  inline DN& operator=(const double x) {
    v = x;
    a = 0;
    return *this;
  }

  inline DN& operator+=(const DN& x) {
    v += x.v;
    a += x.a;
    return *this;
  }

  inline DN& operator/=(const DN& x) {
    a = (a * x.v - x.a * v) / std::pow(x.v, 2.0);
    v /= x.v;
    return *this;
  }

  operator double() const { return v; }
};

inline std::ostream& operator<<(std::ostream& out, const DN& n) {
  out << "(" << n.v << ", " << n.a << ")";
  return out;
}

inline DN operator-(const DN& x) { return DN{-x.v, -x.a}; }
inline DN operator+(const DN& x, const DN& y) { return DN{x.v + y.v, x.a + y.a}; }
inline DN operator-(const DN& x, const DN& y) { return DN{x.v - y.v, x.a - y.a}; }
inline DN operator*(const DN& x, const DN& y) { return DN{x.v * y.v, x.a * y.v + y.a * x.v}; }
inline DN operator/(const DN& x, const DN& y) {
  return DN{x.v / y.v, (x.a * y.v - y.a * x.v) / std::pow(y.v, 2.0)};
}

inline bool operator==(const DN& x, const DN& y) { return x.v == y.v && x.a == y.a; }
inline bool operator==(const DN& x, const double y) { return x.v == y && x.a == 0.0; }
inline bool operator==(const double x, const DN& y) { return x == y.v && 0.0 == y.a; }
inline bool operator!=(const DN& x, const DN& y) { return x.v != y.v || x.a != y.a; }
inline bool operator!=(const DN& x, const double y) { return x.v != y || x.a != 0.0; }
inline bool operator!=(const double x, const DN& y) { return x != y.v || 0.0 != y.a; }
inline bool operator<(const DN& x, const DN& y) { return x.v < y.v; }
inline bool operator<(const DN& x, const double y) { return x.v < y; }
inline bool operator<(const double x, const DN& y) { return x < y; }
inline bool operator>(const DN& x, const DN& y) { return x.v > y.v; }
inline bool operator>(const DN& x, const double y) { return x.v > y; }
inline bool operator>(const double x, const DN& y) { return x > y.v; }
inline bool operator<=(const DN& x, const DN& y) { return x.v <= y.v; }
inline bool operator<=(const DN& x, const double y) { return x.v <= y; }
inline bool operator<=(const double x, const DN& y) { return x <= y.v; }
inline bool operator>=(const DN& x, const DN& y) { return x.v >= y.v; }
inline bool operator>=(const DN& x, const double y) { return x.v >= y; }
inline bool operator>=(const double x, const DN& y) { return x >= y.v; }

inline DN pow(const DN& x, const DN& y) {
  const auto powv = std::pow(x.v, y.v);
  return DN{powv, powv * (std::log(x.v) * y.a + y.v / x.v * x.a)};
}

inline DN pow(const DN& x, const double y) {
  return DN{std::pow(x.v, y), y * std::pow(x.v, y - 1.0) * x.a};
}

inline DN pow(const double x, const DN& y) {
  const auto powv = std::pow(x, y.v);
  return DN{powv, powv * std::log(x) * y.a};
}

inline DN sin(const DN& x) { return DN{std::sin(x.v), x.a * std::cos(x.v)}; }
inline DN cos(const DN& x) { return DN{std::cos(x.v), -x.a * std::sin(x.v)}; }
inline DN tan(const DN& x) {
  const auto tanv = std::tan(x.v);
  return DN{tanv, x.a * (1 + std::pow(tanv, 2.0))};
}

inline DN asin(const DN& x) { return DN{std::asin(x.v), x.a / std::sqrt(1 - std::pow(x.v, 2.0))}; }

inline DN acos(const DN& x) {
  return DN{std::acos(x.v), -x.a / std::sqrt(1 - std::pow(x.v, 2.0))};
}

inline DN atan(const DN& x) {
  return DN{std::atan(x.v), x.a * (1 - std::pow(std::tanh(x.v), 2.0))};
}

inline DN atan2(const DN& y, const DN& x) {
  const DN q = y / x;
  return DN{std::atan2(y.v, x.v), q.a * (1 - std::pow(std::tanh(q.v), 2.0))};
}

inline DN atan2(const DN& y, const double x) {
  return DN{std::atan2(y.v, x), y.a * (1 - std::pow(std::tanh(y.v / x), 2.0))};
}

inline DN atan2(const double x, const DN& y) { return atan2(y, x); }

inline DN exp(const DN& x) { return DN{std::exp(x.v), x.a * std::exp(x.v)}; }
inline DN log(const DN& x) { return DN{std::log(x.v), x.a / x.v}; }
inline DN sqrt(const DN& x) {
  const double v_sqrt = std::sqrt(x.v);
  const double denom  = 2 * v_sqrt;
  return DN{v_sqrt, denom == 0 ? 0.0 : x.a / denom};
}

inline DN abs(const DN& x) { return DN{std::fabs(x.v), std::copysign(x.a, x.v)}; }
inline DN abs2(const DN& x) { return x * x; }
inline DN min(const DN& x, const DN& y) {
  if (std::max(std::fabs(x.v), std::fabs(y.v)) == std::numeric_limits<double>::infinity()) {
    return x <= y ? x : y;
  }

  const auto z = x >= y ? 1.0 : 0.0;
  return DN{z * y.v + (1.0 - z) * x.v, z * y.a + (1.0 - z) * x.a};
}

inline DN max(const DN& x, const DN& y) {
  if (std::max(std::fabs(x.v), std::fabs(y.v)) == std::numeric_limits<double>::infinity()) {
    return x >= y ? x : y;
  }

  const auto z = y >= x ? 1.0 : 0.0;
  return DN{z * y.v + (1.0 - z) * x.v, z * y.a + (1.0 - z) * x.a};
}

inline DN operator+(const DN& x, const double y) { return DN{x.v + y, x.a}; }
inline DN operator+(const double x, const DN& y) { return DN{x + y.v, y.a}; }
inline DN operator-(const DN& x, const double y) { return DN{x.v - y, x.a}; }
inline DN operator-(const double x, const DN& y) { return DN{x - y.v, -y.a}; }
inline DN operator*(const DN& x, const double y) { return DN{x.v * y, x.a * y}; }
inline DN operator*(const double x, const DN& y) { return DN{x * y.v, y.a * x}; }
inline DN operator/(const DN& x, const double y) {
  return DN{x.v / y, (x.a * y) / std::pow(y, 2.0)};
}

inline DN operator/(const double x, const DN& y) {
  return DN{x / y.v, (-y.a * x) / std::pow(y.v, 2.0)};
}

inline DN ceil(const DN& x) { return DN{std::ceil(x.v), 0.0}; }
inline DN floor(const DN& x) { return DN{std::floor(x.v), 0.0}; }

inline DN cosh(const DN& x) { return DN{std::cosh(x.v), x.a * std::sinh(x.v)}; }
inline DN sinh(const DN& x) { return DN{std::sinh(x.v), x.a * std::cosh(x.v)}; }
inline DN tanh(const DN& x) {
  return DN{std::tanh(x.v), x.a * (1.0 - std::pow(std::tanh(x.v), 2))};
}
}  // namespace addn

namespace Eigen {
template <> struct NumTraits<addn::DN> : NumTraits<double> {
  using Real       = addn::DN;
  using NonInteger = addn::DN;
  using Literal    = addn::DN;
  using Nested     = addn::DN;

  enum {
    IsComplex             = 0,
    IsInteger             = 0,
    IsSigned              = 1,
    RequireInitialization = 1,
    ReadCost              = 1,
    AddCost               = 2,
    MulCost               = 4
  };
};
}  // namespace Eigen
