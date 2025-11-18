/*
 * Atmosphere.h
 */

#pragma once

#include "Environment.h"
#include "Constants.h"

#include <cmath>

namespace BulletPhysic {
namespace dynamics {
namespace environment {

// provides atmospheric properties based on altitude (ISA model)
class Atmosphere : public IEnvironment {
public:
    Atmosphere(float groundY = 0.0f) : m_groundY(groundY) {}

    void update(PhysicsContext& context, const RigidBody& rb) override
    {
        float altitude = std::max(0.0f, std::min(rb.position().y - m_groundY, constants::TROPOSPHERE_MAX));

        // linear temperature decrease: T = T0 - L * h
        float temperature = constants::SEA_LEVEL_TEMPERATURE - constants::LAPSE_RATE * altitude;

        // barometric formula: p = p0 * (T / T0)^(g / (R * L)) = p0 * (T / T0)^exp
        float pressure = constants::SEA_LEVEL_PRESSURE * std::pow(temperature / constants::SEA_LEVEL_TEMPERATURE, BAROMETRIC_EXP);

        // ideal gas law: rho = p / (R * T)
        float density = pressure / (constants::GAS_CONSTANT_DRY_AIR * temperature);

        context.temperature = temperature;
        context.pressure = pressure;
        context.density = density;
    }

    const std::string& getName() const override { return m_name; }

private:
    // barometric formula exponent: g / (R * L)
    static inline const float BAROMETRIC_EXP = constants::GRAVITY.length() / (constants::GAS_CONSTANT_DRY_AIR * constants::LAPSE_RATE);

    std::string m_name = "Atmosphere";
    float m_groundY;
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
