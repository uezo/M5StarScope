#include <unity.h>

#include <cmath>
#include <cstdint>
#include <cstring>

#include "starscope/astronomy.hpp"
#include "starscope/city_catalog.hpp"
#include "starscope/fusion.hpp"
#include "starscope/geomagnetism.hpp"
#include "starscope/protocol.hpp"
#include "starscope/quaternion.hpp"
#include "starscope/scope_projection.hpp"
#include "starscope/star_catalog.hpp"

using namespace starscope;

void test_j2000_epoch() {
  CivilTime time{2000, 1, 1, 12, 0, 0};
  TEST_ASSERT_DOUBLE_WITHIN(0.000001, 2451545.0,
                            astronomy::julianDateUtc(time, 0));
  TEST_ASSERT_DOUBLE_WITHIN(0.0001, 280.4606,
                            astronomy::greenwichMeanSiderealDeg(2451545.0));
}

void test_local_time_offset_conversion() {
  CivilTime tokyoNoon{2000, 1, 1, 21, 0, 0};
  TEST_ASSERT_DOUBLE_WITHIN(0.000001, 2451545.0,
                            astronomy::julianDateUtc(tokyoNoon, 540));
}

void test_julian_date_to_utc_round_trip() {
  const CivilTime samples[] = {
      {2000, 1, 1, 12, 0, 0},
      {2024, 2, 29, 23, 59, 59},
      {2026, 8, 5, 13, 3, 32},
      {2099, 12, 31, 0, 0, 0},
  };
  for (const auto& expected : samples) {
    const CivilTime actual =
        astronomy::civilTimeUtc(astronomy::julianDateUtc(expected, 0));
    TEST_ASSERT_EQUAL_INT(expected.year, actual.year);
    TEST_ASSERT_EQUAL_INT(expected.month, actual.month);
    TEST_ASSERT_EQUAL_INT(expected.day, actual.day);
    TEST_ASSERT_EQUAL_INT(expected.hour, actual.hour);
    TEST_ASSERT_EQUAL_INT(expected.minute, actual.minute);
    TEST_ASSERT_EQUAL_INT(expected.second, actual.second);
  }
}

void test_zenith_coordinate() {
  Location location{0.0, 0.0, 0, "Equator"};
  const double jd = 2451545.0;
  const double ra = astronomy::greenwichMeanSiderealDeg(jd);
  const auto horizontal =
      astronomy::equatorialToHorizontal(ra, 0.0, jd, location);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 90.0F, horizontal.altitudeDeg);
}

void test_direct_equatorial_world_vector() {
  const Location location{35.681236, 139.767125, 540, "Tokyo"};
  const double jd = 2461256.0;
  const auto horizontal =
      astronomy::equatorialToHorizontal(279.2347, 38.7837, jd, location);
  const Vec3 direct =
      astronomy::equatorialToWorld(279.2347, 38.7837, jd, location);
  const Vec3 viaAngles = horizontalToWorld(horizontal);
  TEST_ASSERT_FLOAT_WITHIN(0.00001F, viaAngles.x, direct.x);
  TEST_ASSERT_FLOAT_WITHIN(0.00001F, viaAngles.y, direct.y);
  TEST_ASSERT_FLOAT_WITHIN(0.00001F, viaAngles.z, direct.z);
}

void test_solar_and_lunar_ephemeris() {
  // The March equinox places the Sun at RA 0 / Dec 0 (within the precision of
  // this intentionally compact ephemeris).
  const CivilTime equinox{2024, 3, 20, 3, 6, 0};
  const auto equinoxBodies = astronomy::solarSystemEphemeris(
      astronomy::julianDateUtc(equinox, 0));
  TEST_ASSERT_FLOAT_WITHIN(
      0.5F, 0.0F,
      astronomy::signedAngleDifference(
          static_cast<float>(equinoxBodies.sun.rightAscensionDeg), 0.0F));
  TEST_ASSERT_DOUBLE_WITHIN(0.5, 0.0,
                            equinoxBodies.sun.declinationDeg);

  // The 2024-04-08 total solar eclipse is also an excellent new-moon
  // regression point. The 2024-01-25 timestamp is the corresponding full-moon
  // sanity check for the illuminated fraction.
  const CivilTime eclipse{2024, 4, 8, 18, 0, 0};
  const auto newMoon = astronomy::solarSystemEphemeris(
      astronomy::julianDateUtc(eclipse, 0));
  TEST_ASSERT_LESS_THAN_FLOAT(0.01F, newMoon.moonIlluminatedFraction);

  const CivilTime fullMoonTime{2024, 1, 25, 17, 54, 0};
  const auto fullMoon = astronomy::solarSystemEphemeris(
      astronomy::julianDateUtc(fullMoonTime, 0));
  TEST_ASSERT_GREATER_THAN_FLOAT(0.98F, fullMoon.moonIlluminatedFraction);

  // Regression for the original fixed "Sol" bug: at 20:36 JST on 2026-08-04
  // the actual Sun is already well below Tokyo's horizon.
  const CivilTime tokyoNightUtc{2026, 8, 4, 11, 36, 0};
  const double tokyoNightJd = astronomy::julianDateUtc(tokyoNightUtc, 0);
  const auto tokyoNightBodies = astronomy::solarSystemEphemeris(tokyoNightJd);
  const Location tokyo{35.681236, 139.767125, 540, "Tokyo"};
  const auto sunHorizontal = astronomy::equatorialToHorizontal(
      tokyoNightBodies.sun.rightAscensionDeg,
      tokyoNightBodies.sun.declinationDeg, tokyoNightJd, tokyo);
  TEST_ASSERT_LESS_THAN_FLOAT(-10.0F, sunHorizontal.altitudeDeg);
}

void test_naked_eye_planet_ephemeris() {
  // Venus passed less than a degree from the Sun at inferior conjunction on
  // 2020-06-03. This catches the common Earth-vector sign error.
  const CivilTime venusConjunctionUtc{2020, 6, 3, 18, 0, 0};
  const auto conjunction = astronomy::solarSystemEphemeris(
      astronomy::julianDateUtc(venusConjunctionUtc, 0));
  const auto& venus =
      conjunction.planets[static_cast<std::size_t>(PlanetKind::Venus)];
  const float venusSunSeparation = astronomy::angularSeparationDeg(
      static_cast<float>(venus.equatorial.rightAscensionDeg),
      static_cast<float>(venus.equatorial.declinationDeg),
      static_cast<float>(conjunction.sun.rightAscensionDeg),
      static_cast<float>(conjunction.sun.declinationDeg));
  TEST_ASSERT_LESS_THAN_FLOAT(1.0F, venusSunSeparation);

  // Jupiter was at opposition on 2024-12-07, nearly opposite the Sun.
  const CivilTime jupiterOppositionUtc{2024, 12, 7, 20, 0, 0};
  const auto opposition = astronomy::solarSystemEphemeris(
      astronomy::julianDateUtc(jupiterOppositionUtc, 0));
  const auto& jupiter =
      opposition.planets[static_cast<std::size_t>(PlanetKind::Jupiter)];
  const float jupiterSunSeparation = astronomy::angularSeparationDeg(
      static_cast<float>(jupiter.equatorial.rightAscensionDeg),
      static_cast<float>(jupiter.equatorial.declinationDeg),
      static_cast<float>(opposition.sun.rightAscensionDeg),
      static_cast<float>(opposition.sun.declinationDeg));
  TEST_ASSERT_GREATER_THAN_FLOAT(175.0F, jupiterSunSeparation);

  for (std::size_t i = 0; i < kNakedEyePlanetCount; ++i) {
    const auto& planet = opposition.planets[i];
    TEST_ASSERT_EQUAL_UINT8(static_cast<std::uint8_t>(i),
                            static_cast<std::uint8_t>(planet.kind));
    TEST_ASSERT_TRUE(std::isfinite(planet.equatorial.rightAscensionDeg));
    TEST_ASSERT_TRUE(std::isfinite(planet.equatorial.declinationDeg));
    TEST_ASSERT_TRUE(planet.equatorial.rightAscensionDeg >= 0.0);
    TEST_ASSERT_TRUE(planet.equatorial.rightAscensionDeg < 360.0);
    TEST_ASSERT_GREATER_THAN_FLOAT(0.0F, planet.distanceAu);
  }
}

void test_display_star_colors_are_distinct() {
  const auto blue = astronomy::colorFromBv(-0.3F);
  const auto red = astronomy::colorFromBv(1.8F);
  TEST_ASSERT_TRUE(static_cast<int>(blue.blue) - blue.red > 70);
  TEST_ASSERT_TRUE(static_cast<int>(red.red) - red.blue > 100);
}

void test_angle_wrapping() {
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 2.0F,
                           astronomy::signedAngleDifference(1.0F, 359.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.001F, -2.0F,
                           astronomy::signedAngleDifference(359.0F, 1.0F));
}

void test_protocol_round_trip_and_resync() {
  protocol::OrientationPacket packet;
  packet.flags = protocol::kFlagValid | protocol::kFlagCalibrated |
                 protocol::kFlagSetupToggle;
  packet.sequence = 42;
  const Quaternion encoded = quaternion::fromPointingDeg(123.0F, 34.0F, 12.0F);
  packet.quaternionW = static_cast<std::int16_t>(encoded.w * 16384.0F);
  packet.quaternionX = static_cast<std::int16_t>(encoded.x * 16384.0F);
  packet.quaternionY = static_cast<std::int16_t>(encoded.y * 16384.0F);
  packet.quaternionZ = static_cast<std::int16_t>(encoded.z * 16384.0F);
  packet.accuracyPercent = 87;
  protocol::finalize(packet);
  TEST_ASSERT_TRUE(protocol::isValid(packet));

  protocol::OrientationPacketParser parser;
  protocol::OrientationPacket parsed;
  TEST_ASSERT_FALSE(parser.push(0x00, parsed));
  bool completed = false;
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(&packet);
  for (std::size_t i = 0; i < sizeof(packet); ++i) {
    completed = parser.push(bytes[i], parsed) || completed;
  }
  TEST_ASSERT_TRUE(completed);
  TEST_ASSERT_EQUAL_UINT16(42, parsed.sequence);
  TEST_ASSERT_TRUE((parsed.flags & protocol::kFlagSetupToggle) != 0);
  const auto sample = protocol::toSample(parsed, 1000);
  TEST_ASSERT_FLOAT_WITHIN(0.03F, 123.0F, sample.yawDeg);
  TEST_ASSERT_FLOAT_WITHIN(0.03F, 34.0F, sample.pitchDeg);
  TEST_ASSERT_TRUE(sample.valid);
  TEST_ASSERT_TRUE(sample.calibrated);
}

void test_quaternion_scope_basis_and_projection() {
  const Quaternion pose = quaternion::fromPointingDeg(0.0F, 0.0F, 0.0F);
  const ScopeBasis basis = scopeBasis(pose);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, basis.forward.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 1.0F, basis.forward.y);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 1.0F, basis.right.x);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 1.0F, basis.up.z);

  const auto center = projectIntoScope(
      horizontalToWorld({0.0F, 0.0F}), basis, 45.0F);
  TEST_ASSERT_TRUE(center.visible);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, center.normalizedX);
  TEST_ASSERT_FLOAT_WITHIN(0.001F, 0.0F, center.normalizedY);

  const auto right = projectIntoScope(
      horizontalToWorld({20.0F, 0.0F}), basis, 45.0F);
  TEST_ASSERT_TRUE(right.visible);
  TEST_ASSERT_TRUE(right.normalizedX > 0.5F);
}

void test_pose_fusion_initial_reference() {
  PoseFusion fusion;
  // Sensor +X points magnetic north, +Z points up.
  TEST_ASSERT_TRUE(fusion.update({0.0F, 0.0F, 0.0F},
                                 {0.0F, 0.0F, 1.0F},
                                 {30.0F, 0.0F, -20.0F}, 1000));
  float azimuth = 0.0F;
  float altitude = 0.0F;
  float roll = 0.0F;
  quaternion::toEulerDeg(fusion.sensorToMagneticWorld(), azimuth, altitude,
                         roll);
  TEST_ASSERT_FLOAT_WITHIN(
      0.01F, 0.0F, astronomy::signedAngleDifference(azimuth, 0.0F));
  TEST_ASSERT_FLOAT_WITHIN(0.01F, 0.0F, altitude);
}

void test_true_north_correction() {
  const Quaternion magnetic = quaternion::fromPointingDeg(0.0F, 0.0F);
  const Quaternion corrected = applyWorldHeadingCorrection(magnetic, -7.5F);
  float azimuth = 0.0F;
  float altitude = 0.0F;
  float roll = 0.0F;
  quaternion::toEulerDeg(corrected, azimuth, altitude, roll);
  TEST_ASSERT_FLOAT_WITHIN(0.01F, 352.5F, azimuth);
}

void test_wmm2025_geomagnetic_declination() {
  using starscope::geomagnetism::worldMagneticModel2025;
  struct TestValue {
    double year;
    double latitude;
    double longitude;
    double declination;
  };
  // NOAA/NCEI WMM2025 surface test values.
  static constexpr TestValue values[] = {
      {2025.0, 80.0, 0.0, 1.28},     {2025.0, 0.0, 120.0, -0.16},
      {2025.0, -80.0, 240.0, 68.78}, {2027.5, 80.0, 0.0, 2.59},
      {2027.5, 0.0, 120.0, -0.24},   {2027.5, -80.0, 240.0, 68.49},
  };
  for (const auto& value : values) {
    const auto field = worldMagneticModel2025(
        value.latitude, value.longitude, value.year);
    TEST_ASSERT_TRUE(field.valid);
    TEST_ASSERT_DOUBLE_WITHIN(0.02, value.declination, field.declinationDeg);
  }
  TEST_ASSERT_FALSE(worldMagneticModel2025(35.68, 139.77, 2030.0).valid);
}

void test_embedded_catalogues() {
  StarCatalog stars;
  TEST_ASSERT_GREATER_THAN(8500, stars.size());
  TEST_ASSERT_TRUE(stars.isSolarPlaceholder(stars.packed(0)));
  TEST_ASSERT_FALSE(stars.isSolarPlaceholder(stars.packed(1)));
  for (std::size_t i = 1; i < stars.size(); ++i) {
    TEST_ASSERT_TRUE(stars.magnitude(stars.packed(i - 1)) <=
                     stars.magnitude(stars.packed(i)));
  }

  CityCatalog cities;
  TEST_ASSERT_GREATER_THAN(200, cities.countryCount());
  const std::size_t japan = cities.findCountry("JP");
  TEST_ASSERT_LESS_THAN(cities.countryCount(), japan);
  const auto& country = cities.country(japan);
  TEST_ASSERT_GREATER_THAN(40, country.cityCount);
  const std::size_t tokyo = cities.findCityByName(country, "Tokyo");
  TEST_ASSERT_LESS_THAN(country.cityCount, tokyo);
  const auto& tokyoCity = cities.city(country, tokyo);
  TEST_ASSERT_TRUE(cities.matches(country, tokyoCity, "tok"));
  TEST_ASSERT_TRUE(cities.matches(country, tokyoCity, "JAPAN"));
  TEST_ASSERT_TRUE(cities.matches(country, tokyoCity, "jp"));
  TEST_ASSERT_FALSE(cities.matches(country, tokyoCity, "Osaka"));
  const Location location = cities.location(tokyoCity);
  TEST_ASSERT_DOUBLE_WITHIN(0.2, 35.68, location.latitudeDeg);
  TEST_ASSERT_DOUBLE_WITHIN(0.2, 139.77, location.longitudeDeg);
  TEST_ASSERT_EQUAL_INT(540, location.utcOffsetMinutes);
}

int main(int, char**) {
  UNITY_BEGIN();
  RUN_TEST(test_j2000_epoch);
  RUN_TEST(test_local_time_offset_conversion);
  RUN_TEST(test_julian_date_to_utc_round_trip);
  RUN_TEST(test_zenith_coordinate);
  RUN_TEST(test_direct_equatorial_world_vector);
  RUN_TEST(test_solar_and_lunar_ephemeris);
  RUN_TEST(test_naked_eye_planet_ephemeris);
  RUN_TEST(test_display_star_colors_are_distinct);
  RUN_TEST(test_angle_wrapping);
  RUN_TEST(test_protocol_round_trip_and_resync);
  RUN_TEST(test_quaternion_scope_basis_and_projection);
  RUN_TEST(test_pose_fusion_initial_reference);
  RUN_TEST(test_true_north_correction);
  RUN_TEST(test_wmm2025_geomagnetic_declination);
  RUN_TEST(test_embedded_catalogues);
  return UNITY_END();
}
