#pragma once

#include <cstddef>
#include <cstdint>

#include "starscope/types.hpp"

namespace starscope {

struct PackedStar {
  std::uint32_t rightAscensionMilliDeg;
  std::int32_t declinationMilliDeg;
  std::int16_t magnitudeCenti;
  std::int16_t colorIndexMilli;
  std::uint32_t hipId;
  std::uint32_t nameOffset;
};

constexpr std::uint32_t kNoName = 0xFFFFFFFFU;

extern const PackedStar kPackedStars[];
extern const std::size_t kPackedStarCount;
extern const char kStarNamePool[];

class StarCatalog {
 public:
  std::size_t size() const { return kPackedStarCount; }
  const PackedStar& packed(std::size_t index) const { return kPackedStars[index]; }
  const char* name(const PackedStar& star) const {
    return star.nameOffset == kNoName ? nullptr : kStarNamePool + star.nameOffset;
  }
  double rightAscensionDeg(const PackedStar& star) const {
    return star.rightAscensionMilliDeg / 1000.0;
  }
  double declinationDeg(const PackedStar& star) const {
    return star.declinationMilliDeg / 1000.0;
  }
  float magnitude(const PackedStar& star) const {
    return star.magnitudeCenti / 100.0F;
  }
  float colorIndex(const PackedStar& star) const {
    return star.colorIndexMilli / 1000.0F;
  }
  bool isSolarPlaceholder(const PackedStar& star) const {
    return star.hipId == 0U && star.rightAscensionMilliDeg == 0U &&
           star.declinationMilliDeg == 0 && star.magnitudeCenti < -2000;
  }
};

struct StarSelection {
  StarView stars[16];
  std::size_t count = 0;
};

class FixedStarProvider {
 public:
  explicit FixedStarProvider(const StarCatalog& catalog) : catalog_(catalog) {}

  StarSelection selectDirection(double julianDate, const Location& location,
                                float azimuthDeg, float azimuthHalfWidthDeg,
                                float limitingMagnitude,
                                std::size_t maximum = 12) const;

  StarSelection selectPointing(double julianDate, const Location& location,
                               float azimuthDeg, float altitudeDeg,
                               float radiusDeg, float limitingMagnitude,
                               std::size_t maximum = 8) const;

 private:
  const StarCatalog& catalog_;
};

}  // namespace starscope
