/*
 * Constants.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysic {
namespace constants {

// physical
inline math::Vec3 GRAVITY{0.0f, -9.80665f, 0.0f};

// conversion
inline constexpr float CELSIUS_TO_KELVIN = 273.15f;     // between Celsius and Kelvin

// atmospheric constants (ISA model)
inline constexpr float TROPOSPHERE_MAX = 11000.0f;                      // m (troposphere height)
inline constexpr float BASE_TEMPERATURE = 15 + CELSIUS_TO_KELVIN;       // K (15 C)
inline constexpr float BASE_ATMOSPHERIC_PRESSURE = 101325.0f;           // Pa
inline constexpr float BASE_ATMOSPHERIC_DENSITY = 1.225f;               // kg/m^3 (dry air)
inline constexpr float LAPSE_RATE = 0.0065f;                            // K/m (temperature lapse rate)
inline constexpr float GAS_CONSTANT_DRY_AIR = 287.058f;                 // J/(kg·K)

// humidity constants (Tetens)
inline constexpr float GAS_CONSTANT_WATER_VAPOR = 461.495f;             // J/(kg·K)
inline constexpr float TETENS_A = 17.27f;
inline constexpr float TETENS_B = 35.85f;
inline constexpr float TETENS_C = 0.61078f;

// Earth (WGS standard)
inline constexpr double EARTH_ANGULAR_SPEED = 72.92115e-6;              // rad/s
inline constexpr double EARTH_GRAVITATIONAL_CONSTANT = 3.986004418e14;  // GM (m^3/s^2)
inline constexpr double EARTH_SEMI_MAJOR_AXIS = 6378137.0;              // m (a, or equatorial radius)
inline constexpr double EARTH_SEMI_MINOR_AXIS = 6356752.314245;         // m (b, or polar radius)
inline constexpr double EARTH_ECCENTRICITY_SQUARED = 6.69437999014e-3;  // e^2

// default rigid body (sphere) constants
static constexpr float DEFAULT_SPHERE_AREA = 0.01f;     // m^2
static constexpr float DEFAULT_SPHERE_CD = 0.47f;

} // namespace constants
} // namespace BulletPhysic