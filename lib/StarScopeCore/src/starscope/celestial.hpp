#pragma once

#include <cstddef>

#include "starscope/types.hpp"

namespace starscope {

enum class CelestialKind { FixedStar, Sun, Moon, Planet };

struct CelestialObject {
  CelestialKind kind = CelestialKind::FixedStar;
  const char* name = nullptr;
  float magnitude = 0.0F;
  float colorIndex = 0.65F;
  HorizontalCoordinate horizontal;
};

// Providers can expose fixed and moving objects without coupling a view to a
// particular catalogue or ephemeris implementation.
class CelestialObjectProvider {
 public:
  virtual ~CelestialObjectProvider() = default;
  virtual std::size_t objects(double julianDate, const Location& location,
                              CelestialObject* output,
                              std::size_t capacity) const = 0;
};

}  // namespace starscope
