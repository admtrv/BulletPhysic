/*
 * Vec3.cpp
 */

#include "Vec3.h"

namespace BulletPhysic {
namespace math {

Vec3::Vec3() : x(0.0f), y(0.0f), z(0.0f) {}
Vec3::Vec3(float X, float Y, float Z) : x(X), y(Y), z(Z) {}

Vec3 Vec3::operator+(const Vec3& rhs) const
{
    return {x + rhs.x, y + rhs.y, z + rhs.z};
}
Vec3 Vec3::operator-(const Vec3& rhs) const
{
    return {x - rhs.x, y - rhs.y, z - rhs.z};
}
Vec3 Vec3::operator*(float scalar) const
{
    return {x * scalar, y * scalar, z * scalar};
}
Vec3 Vec3::operator/(float scalar) const
{
    return {x / scalar, y / scalar, z / scalar};
}

Vec3& Vec3::operator+=(const Vec3& rhs)
{
    x += rhs.x;
    y += rhs.y;
    z += rhs.z;
    return *this;
}
Vec3& Vec3::operator-=(const Vec3& rhs)
{
    x -= rhs.x;
    y -= rhs.y;
    z -= rhs.z;
    return *this;
}
Vec3& Vec3::operator*=(float scalar)
{
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
}

Vec3 operator*(float scalar, const Vec3& vec)
{
    return vec * scalar;
}

} // namespace math
} // namespace BulletPhysic
