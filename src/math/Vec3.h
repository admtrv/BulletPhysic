/*
 * Vec3.h
 */

#pragma once

namespace BulletPhysic {
namespace math {

struct Vec3 {
    float x, y, z;

    Vec3();
    Vec3(float X, float Y, float Z);

    Vec3 operator+(const Vec3& rhs) const;
    Vec3 operator-(const Vec3& rhs) const;
    Vec3 operator*(float scalar) const;
    Vec3 operator/(float scalar) const;

    Vec3& operator+=(const Vec3& rhs);
    Vec3& operator-=(const Vec3& rhs);
    Vec3& operator*=(float scalar);
};

Vec3 operator*(float scalar, const Vec3& vec);

} // namespace math
} // namespace BulletPhysic
