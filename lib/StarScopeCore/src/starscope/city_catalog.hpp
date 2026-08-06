#pragma once

#include <cstddef>
#include <cstdint>

#include "starscope/types.hpp"

namespace starscope {

struct PackedCity {
  std::int32_t latitudeE5;
  std::int32_t longitudeE5;
  std::uint32_t nameOffset;
  std::int16_t standardUtcOffsetMinutes;
  std::int16_t daylightUtcOffsetMinutes;
};

struct PackedCountry {
  char code[3];
  std::uint32_t nameOffset;
  std::uint16_t firstCity;
  std::uint16_t cityCount;
};

extern const PackedCity kPackedCities[];
extern const std::size_t kPackedCityCount;
extern const PackedCountry kPackedCountries[];
extern const std::size_t kPackedCountryCount;
extern const char kPlaceNamePool[];

class CityCatalog {
 public:
  std::size_t countryCount() const { return kPackedCountryCount; }
  const PackedCountry& country(std::size_t index) const {
    return kPackedCountries[index];
  }
  const PackedCity& city(const PackedCountry& country,
                         std::size_t cityIndex) const {
    return kPackedCities[country.firstCity + cityIndex];
  }
  const char* countryName(const PackedCountry& country) const {
    return kPlaceNamePool + country.nameOffset;
  }
  const char* cityName(const PackedCity& city) const {
    return kPlaceNamePool + city.nameOffset;
  }
  Location location(const PackedCity& city, bool daylightTime = false) const {
    Location result;
    result.latitudeDeg = city.latitudeE5 / 100000.0;
    result.longitudeDeg = city.longitudeE5 / 100000.0;
    result.utcOffsetMinutes = daylightTime ? city.daylightUtcOffsetMinutes
                                           : city.standardUtcOffsetMinutes;
    result.name = cityName(city);
    return result;
  }

  std::size_t findCountry(const char* isoCode) const;
  std::size_t findCityByName(const PackedCountry& country,
                             const char* name) const;
  bool matches(const PackedCountry& country, const PackedCity& city,
               const char* query) const;
};

}  // namespace starscope
