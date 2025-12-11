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

// provides atmospheric properties based on altitude (ISA model for Troposphere)
class Atmosphere : public IEnvironment {
public:
    explicit Atmosphere(
        float baseTemperature = constants::BASE_TEMPERATURE,
        float basePressure = constants::BASE_ATMOSPHERIC_PRESSURE,
        float groundY = 0.0f)
            : m_baseTemperature(baseTemperature)
            , m_basePressure(basePressure)
            , m_groundY(groundY)
    {}

    void update(IPhysicsBody& body, PhysicsContext& context) override
    {
        float altitude = std::max(0.0f, std::min(body.getPosition().y - m_groundY, constants::TROPOSPHERE_MAX));

        // linear temperature decrease: T = T0 - L * h
        float temperature = m_baseTemperature - constants::LAPSE_RATE * altitude;

        // barometric formula: p = p0 * (T / T0)^(g / (R * L))
        float pressure = m_basePressure * std::pow(temperature / m_baseTemperature, BAROMETRIC_EXP);

        // ideal gas law: rho = p / (R * T)
        float density = pressure / (constants::GAS_CONSTANT_DRY_AIR * temperature);

        context.airTemperature = temperature;
        context.airPressure = pressure;
        context.airDensity = density;
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Atmosphere";

    // barometric formula exponent: g / (R * L)
    static inline const float BAROMETRIC_EXP = constants::GRAVITY.length() / (constants::GAS_CONSTANT_DRY_AIR * constants::LAPSE_RATE);

    float m_groundY;
    float m_baseTemperature;      // K
    float m_basePressure;         // Pa
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
