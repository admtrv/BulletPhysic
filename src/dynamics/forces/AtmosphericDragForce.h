/*
 * AtmosphericDragForce.h
 */

#pragma once

#include "Force.h"
#include "DragForce.h"
#include "Constants.h"

#include <cmath>
#include <memory>

namespace BulletPhysic {
namespace dynamics {
namespace forces {

// interface for atmosphere models
class IAtmosphereModel {
public:
    virtual ~IAtmosphereModel() = default;

    // get air density at given altitude (meters)
    virtual float getDensity(float altitude) const = 0;

    // get temperature at given altitude (Kelvin)
    virtual float getTemperature(float altitude) const = 0;

    // get pressure at given altitude (Pascals)
    virtual float getPressure(float altitude) const = 0;
};

// International Standard Atmosphere (ISA) model
class ISAModel : public IAtmosphereModel {
public:
    ISAModel() = default;

    float getDensity(float altitude) const override
    {
        altitude = clampAltitude(altitude);

        float temperature = getTemperature(altitude);
        float pressure = getPressure(altitude);

        // ideal gas law
        return pressure / (constants::GAS_CONSTANT_DRY_AIR * temperature);
    }

    // linear temperature decrease: T = T0 - L * h
    float getTemperature(float altitude) const override
    {
        altitude = clampAltitude(altitude);

        return constants::SEA_LEVEL_TEMPERATURE - constants::LAPSE_RATE * altitude;
    }

    // barometric formula: p = p0 * (1 - L * h / T0)^(g / (R * L)) = p0 * (1 - L * h)^exponent
    float getPressure(float altitude) const override
    {
        altitude = clampAltitude(altitude);

        return constants::SEA_LEVEL_PRESSURE * std::pow(1.0f - constants::LAPSE_RATE * altitude / constants::SEA_LEVEL_TEMPERATURE, BAROMETRIC_EXPONENT);
    }

private:
    // barometric formula exponent: g / (R * L)
    static inline const float BAROMETRIC_EXPONENT = constants::GRAVITY.length() / (constants::GAS_CONSTANT_DRY_AIR * constants::LAPSE_RATE);

    // ISA model for troposphere (up to 11km)
    static constexpr float TROPOSPHERE_MAX_ALTITUDE = 11000.0f; // m
    static float clampAltitude(float altitude)
    {
        if (altitude < 0.0f)
        {
            return 0.0f;
        }

        if (altitude > TROPOSPHERE_MAX_ALTITUDE)
        {
            return TROPOSPHERE_MAX_ALTITUDE;
        }

        return altitude;
    }
};

// exponential atmosphere model: rho = rho0 * e^(-h / H)
class ExponentialAtmosphere : public IAtmosphereModel {
public:

    explicit ExponentialAtmosphere(float scaleHeight = DEFAULT_SCALE_HEIGHT) : m_scaleHeight(scaleHeight) {}

    // exponential density decay: rho = rho0 * e^(-h / H)
    float getDensity(float altitude) const override
    {
        altitude = clampAltitude(altitude);

        return constants::SEA_LEVEL_DENSITY * std::exp(-altitude / m_scaleHeight);
    }

    // isothermal: T = const
    float getTemperature(float altitude) const override
    {
        return constants::SEA_LEVEL_TEMPERATURE;
    }

    // exponential pressure decay: p = p0 * e^(-h / H)
    float getPressure(float altitude) const override
    {
        altitude = clampAltitude(altitude);

        return constants::SEA_LEVEL_PRESSURE * std::exp(-altitude / m_scaleHeight);
    }

private:
    static constexpr float DEFAULT_SCALE_HEIGHT = 8500.0f;  // m

    float m_scaleHeight; // m (atmospheric scale height)

    // ensure altitude is not negative
    static float clampAltitude(float altitude)
    {
        return (altitude < 0.0f) ? 0.0f : altitude;
    }
};

// drag force with atmosphere model
class AtmosphericDragForce : public IForce {
public:
    AtmosphericDragForce(
        std::unique_ptr<IDragModel> dragModel = std::make_unique<QuadraticDrag>(),
        std::unique_ptr<IAtmosphereModel> atmosphere = std::make_unique<ISAModel>(),
        float area = constants::DEFAULT_SPHERE_AREA,
        float groundY = 0.0f)
            : m_dragModel(std::move(dragModel))
            , m_atmosphere(std::move(atmosphere))
            , m_area(area)
            , m_groundY(groundY)
    {}

    // drag force: F_drag = f(v, A, rho)
    void apply(RigidBody& rb, float /*dt*/) override
    {
        if (!m_dragModel || !m_atmosphere)
        {
            return;
        }

        // calculate altitude from ground level
        float altitude = rb.position().y - m_groundY;

        // get air density at current altitude
        float airDensity = m_atmosphere->getDensity(altitude);

        // calculate drag with altitude-dependent density
        math::Vec3 dragForce = m_dragModel->calculate(rb.velocity(), m_area, airDensity);

        rb.addForce(dragForce);
    }


    void setGround(float ground) { m_groundY = ground; }
    void setArea(float area) { m_area = area; }

    float getAirDensityAt(float altitude) const
    {
        return m_atmosphere ? m_atmosphere->getDensity(altitude) : constants::SEA_LEVEL_DENSITY;
    }

    const std::string& getName() const override { return m_name; }

private:
    std::string m_name = "Atmospheric Drag";
    std::unique_ptr<IDragModel> m_dragModel;
    std::unique_ptr<IAtmosphereModel> m_atmosphere;
    float m_area;
    float m_groundY;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic