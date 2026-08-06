#include "starscope/star_catalog.hpp"

#include <algorithm>
#include <cmath>

#include "starscope/astronomy.hpp"

namespace starscope {
namespace {

void insertByMagnitude(StarSelection& selection, const StarView& candidate,
                       std::size_t maximum) {
  maximum = std::min(maximum,
                     sizeof(selection.stars) / sizeof(selection.stars[0]));
  std::size_t insertion = 0;
  while (insertion < selection.count &&
         selection.stars[insertion].magnitude <= candidate.magnitude) {
    ++insertion;
  }
  if (insertion >= maximum) return;
  const std::size_t newCount = std::min(selection.count + 1, maximum);
  for (std::size_t i = newCount - 1; i > insertion; --i) {
    selection.stars[i] = selection.stars[i - 1];
  }
  selection.stars[insertion] = candidate;
  selection.count = newCount;
}

void insertByPointing(StarSelection& selection, const StarView& candidate,
                      float separation, std::size_t maximum,
                      float separations[16]) {
  maximum = std::min(maximum,
                     sizeof(selection.stars) / sizeof(selection.stars[0]));
  std::size_t insertion = 0;
  while (insertion < selection.count && separations[insertion] <= separation) {
    ++insertion;
  }
  if (insertion >= maximum) return;
  const std::size_t newCount = std::min(selection.count + 1, maximum);
  for (std::size_t i = newCount - 1; i > insertion; --i) {
    selection.stars[i] = selection.stars[i - 1];
    separations[i] = separations[i - 1];
  }
  selection.stars[insertion] = candidate;
  separations[insertion] = separation;
  selection.count = newCount;
}

StarView unpack(const StarCatalog& catalog, const PackedStar& packed,
                HorizontalCoordinate horizontal) {
  StarView result;
  result.hipId = packed.hipId;
  result.name = catalog.name(packed);
  result.magnitude = catalog.magnitude(packed);
  result.colorIndex = catalog.colorIndex(packed);
  result.horizontal = horizontal;
  return result;
}

}  // namespace

StarSelection FixedStarProvider::selectDirection(
    double julianDate, const Location& location, float azimuthDeg,
    float azimuthHalfWidthDeg, float limitingMagnitude,
    std::size_t maximum) const {
  StarSelection result;
  for (std::size_t i = 0; i < catalog_.size(); ++i) {
    const auto& star = catalog_.packed(i);
    if (catalog_.isSolarPlaceholder(star)) continue;
    if (catalog_.magnitude(star) > limitingMagnitude) break;
    const auto horizontal = astronomy::equatorialToHorizontal(
        catalog_.rightAscensionDeg(star), catalog_.declinationDeg(star),
        julianDate, location);
    if (horizontal.altitudeDeg <= 0.0F) continue;
    if (std::fabs(astronomy::signedAngleDifference(horizontal.azimuthDeg,
                                                   azimuthDeg)) >
        azimuthHalfWidthDeg) {
      continue;
    }
    insertByMagnitude(result, unpack(catalog_, star, horizontal), maximum);
  }
  return result;
}

StarSelection FixedStarProvider::selectPointing(
    double julianDate, const Location& location, float azimuthDeg,
    float altitudeDeg, float radiusDeg, float limitingMagnitude,
    std::size_t maximum) const {
  StarSelection result;
  float separations[16]{};
  for (std::size_t i = 0; i < catalog_.size(); ++i) {
    const auto& star = catalog_.packed(i);
    if (catalog_.isSolarPlaceholder(star)) continue;
    if (catalog_.magnitude(star) > limitingMagnitude) break;
    const auto horizontal = astronomy::equatorialToHorizontal(
        catalog_.rightAscensionDeg(star), catalog_.declinationDeg(star),
        julianDate, location);
    if (horizontal.altitudeDeg <= -1.0F) continue;
    const float separation = astronomy::angularSeparationDeg(
        horizontal.azimuthDeg, horizontal.altitudeDeg, azimuthDeg, altitudeDeg);
    if (separation > radiusDeg) continue;
    insertByPointing(result, unpack(catalog_, star, horizontal), separation,
                     maximum, separations);
  }
  return result;
}

}  // namespace starscope
