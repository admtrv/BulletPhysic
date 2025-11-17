/*
 * PhysicsContext.h
 */

#pragma once

#include "math/Vec3.h"

#include <optional>

namespace BulletPhysic {
namespace dynamics {

// shared context for physics simulation (environment provides, forces consume)
class PhysicsContext {
public:
    std::optional<float> density;       // kg/m^3
    std::optional<float> temperature;   // K
    std::optional<float> pressure;      // Pa
    std::optional<math::Vec3> wind;     // m/s

    void reset()
    {
        density.reset();
        temperature.reset();
        pressure.reset();
        wind.reset();
    }
};

} // namespace dynamics
} // namespace BulletPhysic
