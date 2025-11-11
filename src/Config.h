/*
 * Config.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysic {
namespace config {

inline constexpr float freeFall = 9.81f;
inline math::Vec3 gravityVec{0.0f, -freeFall, 0.0f};
inline constexpr float ground = 0.0f;

} // namespace config
} // namespace BulletPhysic
