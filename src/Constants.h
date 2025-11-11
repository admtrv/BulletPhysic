/*
 * Constants.h
 */

#pragma once

namespace BulletPhysic {
namespace constants {

// atmospheric constants
inline constexpr float SEA_LEVEL_TEMPERATURE = 288.15f;   // K
inline constexpr float SEA_LEVEL_PRESSURE = 101325.0f;    // Pa
inline constexpr float SEA_LEVEL_DENSITY = 1.225f;        // kg/m^3
inline constexpr float LAPSE_RATE = 0.0065f;              // K/m
inline constexpr float GAS_CONSTANT_DRY_AIR = 287.05f;    // J/(kg·K)

// default rigid body (sphere) constants
static constexpr float DEFAULT_SPHERE_AREA      = 0.01f;      // m^2
static constexpr float DEFAULT_SPHERE_CD        = 0.47f;
static constexpr float DEFAULT_SPHERE_LINEAR_B  = 1.93e-5f;   // kg/s

} // namespace constants
} // namespace BulletPhysic