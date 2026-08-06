#include "starscope/city_catalog.hpp"

#include <cctype>
#include <cstring>

namespace starscope {
namespace {

bool containsCaseInsensitive(const char* text, const char* query) {
  if (!text || !query) return false;
  if (!query[0]) return true;
  for (const char* start = text; *start; ++start) {
    const char* candidate = start;
    const char* expected = query;
    while (*candidate && *expected &&
           std::tolower(static_cast<unsigned char>(*candidate)) ==
               std::tolower(static_cast<unsigned char>(*expected))) {
      ++candidate;
      ++expected;
    }
    if (!*expected) return true;
  }
  return false;
}

}  // namespace

std::size_t CityCatalog::findCountry(const char* isoCode) const {
  for (std::size_t i = 0; i < countryCount(); ++i) {
    if (std::strncmp(country(i).code, isoCode, 2) == 0) return i;
  }
  return countryCount();
}

std::size_t CityCatalog::findCityByName(const PackedCountry& selectedCountry,
                                        const char* name) const {
  for (std::size_t i = 0; i < selectedCountry.cityCount; ++i) {
    if (std::strcmp(cityName(city(selectedCountry, i)), name) == 0) return i;
  }
  return selectedCountry.cityCount;
}

bool CityCatalog::matches(const PackedCountry& selectedCountry,
                          const PackedCity& selectedCity,
                          const char* query) const {
  return containsCaseInsensitive(cityName(selectedCity), query) ||
         containsCaseInsensitive(countryName(selectedCountry), query) ||
         containsCaseInsensitive(selectedCountry.code, query);
}

}  // namespace starscope
