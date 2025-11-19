/*
 * Geographic.h
 * Provides geographic context (lat/lon/alt) and calculates gravity using GM/r²
 */

#pragma once

#include "Environment.h"
#include "geography/Coordinates.h"

#include <cmath>

namespace BulletPhysic {
namespace dynamics {
namespace environment {

// provides geographic context and gravity corrections based on actual position from Earth center
class Geographic : public IEnvironment {
public:
    explicit Geographic(double referenceLatitude, double referenceLongitude, float groundY = 0.0f)
        : m_reference(referenceLatitude, referenceLongitude, 0.0)
        , m_groundY(groundY)
    {}

    void update(PhysicsContext& context, const RigidBody& rb) override
    {
        // calculate altitude above ground level
        float altitudeAbove = std::max(0.0f, rb.position().y - m_groundY);

        // store geographic information
        context.latitude = m_reference.latitude;
        context.longitude = m_reference.longitude;
        context.altitude = static_cast<double>(altitudeAbove);

        // correct gravity
        geography::GeographicPosition currentPosition(m_reference.latitude, m_reference.longitude, altitudeAbove);

        context.gravity = geography::Coordinates::gravitationalAccelerationAtGeodetic(currentPosition);
    }

    const std::string& getName() const override { return m_name; }

    double getReferenceLatitude() const { return m_reference.latitude; }
    double getReferenceLongitude() const { return m_reference.longitude; }

private:
    std::string m_name = "Geographic";
    geography::GeographicPosition m_reference;
    float m_groundY;
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
