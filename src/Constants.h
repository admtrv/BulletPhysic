/*
 * Constants.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysic {
namespace constants {

// physical constants
inline math::Vec3 GRAVITY{0.0f, -9.81f, 0.0f};

// atmospheric constants (ISA model)
inline constexpr float SEA_LEVEL_TEMPERATURE = 288.15f;   // K (15°C)
inline constexpr float SEA_LEVEL_PRESSURE = 101325.0f;    // Pa
inline constexpr float SEA_LEVEL_DENSITY = 1.225f;        // kg/m^3 (dry air)
inline constexpr float LAPSE_RATE = 0.0065f;              // K/m (temperature lapse rate)
inline constexpr float GAS_CONSTANT_DRY_AIR = 287.05f;    // J/(kg·K)
inline constexpr float TROPOSPHERE_MAX = 11000.0f;        // m (troposphere height)

// default rigid body (sphere) constants
static constexpr float DEFAULT_SPHERE_AREA = 0.01f;      // m^2
static constexpr float DEFAULT_SPHERE_CD = 0.47f;

} // namespace constants
} // namespace BulletPhysic