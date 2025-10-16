/*
 * Config.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysic {
namespace config {

inline math::Vec3 gravity{0.0f, -9.81f, 0.0f};
inline float groundY = 0.0f;

} // namespace config
} // namespace BulletPhysic
