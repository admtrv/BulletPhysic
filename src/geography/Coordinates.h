/*
 * Coordinates.h
 */

#pragma once

#include "math/Vec3.h"
#include "Constants.h"

namespace BulletPhysic {
namespace geography {

// Geodetic coordinates
struct GeographicPosition {
    double latitude;   // radians
    double longitude;  // radians
    double altitude;   // meters above

    GeographicPosition() : latitude(0.0), longitude(0.0), altitude(0.0) {}
    GeographicPosition(double lat, double lon, double alt) : latitude(lat), longitude(lon), altitude(alt) {}
};

// ECEF coordinates (Earth-Centered, Earth-Fixed)
struct ECEFPosition {
    double x, y, z;

    ECEFPosition() : x(0.0), y(0.0), z(0.0) {}
    ECEFPosition(double x, double y, double z) : x(x), y(y), z(z) {}

    double r() const;
};

// geographic coordinate conversions ECEF <-> Geodetic <-> ENU (classic local 3D coordinates, OpenGL like, Physics like)
class Coordinates {
public:
    // convert geodetic (lat, lon, alt) to ECEF (x, y, z)
    static ECEFPosition geodeticToECEF(const GeographicPosition& geodetic);

    // convert ECEF (x, y, z) to geodetic (lat, lon, alt)
    static GeographicPosition ecefToGeodetic(const ECEFPosition& ecef);

    // convert ECEF to local ENU (East-North-Up) relative to reference point
    static math::Vec3 ecefToENU(const ECEFPosition& point, const GeographicPosition& reference);

    // convert local ENU to ECEF relative to reference point
    static ECEFPosition enuToECEF(const math::Vec3& enu, const GeographicPosition& reference);

    // calculate gravitational acceleration at ECEF position
    static double gravitationalAcceleration(const ECEFPosition& position);

    // calculate gravitational acceleration at geodetic position
    static double gravitationalAccelerationAtGeodetic(const GeographicPosition& position);
};

} // namespace geography
} // namespace BulletPhysic
