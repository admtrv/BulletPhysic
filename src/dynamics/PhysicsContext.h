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
    std::optional<float> airDensity;        // kg/m^3
    std::optional<float> airTemperature;    // K
    std::optional<float> airPressure;       // Pa
    std::optional<float> airHumidity;       // % (relative humidity 0-100)
    std::optional<math::Vec3> wind;         // m/s

    std::optional<double> latitude;         // rad
    std::optional<double> longitude;        // rad
    std::optional<double> altitude;         // m (above sea level)

    std::optional<float> gravity;           // m/s^2 (gravity acceleration magnitude)

    void reset()
    {
        airDensity.reset();
        airTemperature.reset();
        airPressure.reset();
        airHumidity.reset();
        wind.reset();

        latitude.reset();
        longitude.reset();
        altitude.reset();

        gravity.reset();
    }
};

} // namespace dynamics
} // namespace BulletPhysic
