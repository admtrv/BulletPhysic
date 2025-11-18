/*
 * Humidity.h
 */

#pragma once

#include "Environment.h"
#include "Constants.h"

#include <cmath>

namespace BulletPhysic {
namespace dynamics {
namespace environment {

// provides humidity correction to air density
class Humidity : public IEnvironment {
public:
    explicit Humidity(float relativeHumidity = 50.0f)
        : m_relativeHumidity(relativeHumidity)
    {
        m_relativeHumidity = std::max(0.0f, std::min(100.0f, relativeHumidity));
    }

    void update(PhysicsContext& context, const RigidBody& /*rb*/) override
    {
        // store relative humidity in context
        context.humidity = m_relativeHumidity;

        // density correction requires pressure and temperature from Atmosphere
        if (!context.pressure.has_value() || !context.temperature.has_value() || !context.density.has_value())
        {
            return;
        }

        float temperature = *context.temperature;
        float pressure = *context.pressure;
        float density = *context.density;

        // apply humidity correction
        context.density = correctDensityForHumidity(density, temperature, pressure, m_relativeHumidity);
    }

    const std::string& getName() const override { return m_name; }

private:
    // Tetens approximation: p_sat = 0.61078 * exp((17.27 * (T - 273.15)) / (T - 35.85))
    static float saturationVaporPressure(float tempK)
    {
        float tempC = tempK - constants::CELSIUS_TO_KELVIN;
        float exponent = constants::TETENS_A * tempC / (tempK + constants::TETENS_B);
        return constants::TETENS_C * std::exp(exponent);
    }

    // correct air density for humidity
    // rho_humid = rho_dry + rho_vapor
    // where rho_dry = p_dry / (R_dry * T) and rho_vap = p_vap / (R_vap * T)
    static float correctDensityForHumidity(float rhoDry, float tempK, float pressure, float humidityPercent)
    {
        // saturation vapor pressure
        float pressureSaturation = saturationVaporPressure(tempK);

        // water vapor pressure: p_vap = phi * p_sat, where phi is relative humidity
        float pressureVapor = (humidityPercent / 100.0f) * pressureSaturation;

        float densityVapor = pressureVapor / (constants::GAS_CONSTANT_WATER_VAPOR * tempK);

        return rhoDry + densityVapor;
    }

    std::string m_name = "Humidity";
    float m_relativeHumidity; // % (0-100)
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
