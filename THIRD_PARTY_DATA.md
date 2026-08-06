# Third-party data

## HYG Database 4.1

The embedded fixed-star catalogue is a filtered and quantized derivative of the
HYG Database 4.1 by David Nash / Astronexus. It is licensed under the Creative
Commons Attribution-ShareAlike 4.0 International license.

- Source: https://codeberg.org/astronexus/hyg
- Archived source: https://github.com/astronexus/HYG-Database
- License: https://creativecommons.org/licenses/by-sa/4.0/

The generated derivative contains stars with apparent visual magnitude <= 6.5
and only the fields needed by this firmware.

## GeoNames

The embedded country/city catalogue is a filtered and quantized derivative of
GeoNames data, licensed under Creative Commons Attribution 4.0.

- Source: https://www.geonames.org/
- Download: https://download.geonames.org/export/dump/
- License: https://creativecommons.org/licenses/by/4.0/

## World Magnetic Model 2025

The firmware embeds the degree/order-12 WMM2025 Gauss coefficients published
by NOAA's National Centers for Environmental Information and the British
Geological Survey. The model is valid for 2025.0 through 2030.0 and is used to
estimate magnetic declination from latitude, longitude, and date.

- Source: https://www.ncei.noaa.gov/products/world-magnetic-model
- Model DOI: https://doi.org/10.25921/aqfd-sd83
- Status: U.S. Government material; NOAA states that the WMM source code and
  model may be used freely by the public.
