/*
 * BoxCollider.cpp
 */

#include "BoxCollider.h"

namespace BulletPhysics {
namespace collision {

BoxCollider::BoxCollider(const math::Vec3& size) : m_size(size) {}

void BoxCollider::setAxes(const math::Vec3& axisX, const math::Vec3& axisY, const math::Vec3& axisZ)
{
    m_axes[0] = axisX;
    m_axes[1] = axisY;
    m_axes[2] = axisZ;
}

bool BoxCollider::testCollision(const Collider& other, CollisionInfo& outInfo) const
{
    switch (other.getShape()) {
        case CollisionShape::Box: {
            return testCollisionWithBox(dynamic_cast<const BoxCollider&>(other), outInfo);
        }
        case CollisionShape::Ground: {
            return testCollisionWithGround(dynamic_cast<const GroundCollider&>(other), outInfo);
        }
        default:
            return false;
    }
}

bool BoxCollider::testPoint(const math::Vec3& point) const
{
    math::Vec3 half = m_size * 0.5f;
    math::Vec3 diff = point - m_position;

    return std::abs(diff.x) <= half.x && std::abs(diff.y) <= half.y && std::abs(diff.z) <= half.z;
}

bool BoxCollider::testCollisionWithBox(const BoxCollider& other, CollisionInfo& outInfo) const
{
    math::Vec3 half1 = m_size * 0.5f;
    math::Vec3 half2 = other.m_size * 0.5f;

    math::Vec3 diff = other.m_position - m_position;

    if (std::abs(diff.x) > half1.x + half2.x || std::abs(diff.y) > half1.y + half2.y || std::abs(diff.z) > half1.z + half2.z)
    {
        return false;
    }

    // find axis of minimum penetration
    float minPenetration = std::abs(diff.x) - (half1.x + half2.x);
    outInfo.normal = diff.x > 0.0f ? math::Vec3{1.0f, 0.0f, 0.0f} : math::Vec3{-1.0f, 0.0f, 0.0f};
    outInfo.penetration = -minPenetration;

    float penY = std::abs(diff.y) - (half1.y + half2.y);
    if (-penY < outInfo.penetration)
    {
        outInfo.penetration = -penY;
        outInfo.normal = diff.y > 0.0f ? math::Vec3{0.0f, 1.0f, 0.0f} : math::Vec3{0.0f, -1.0f, 0.0f};
    }

    float penZ = std::abs(diff.z) - (half1.z + half2.z);
    if (-penZ < outInfo.penetration)
    {
        outInfo.penetration = -penZ;
        outInfo.normal = diff.z > 0.0f ? math::Vec3{0.0f, 0.0f, 1.0f} : math::Vec3{0.0f, 0.0f, -1.0f};
    }

    return true;
}

bool BoxCollider::testCollisionWithGround(const GroundCollider& ground, CollisionInfo& outInfo) const
{
    float groundY = ground.getGroundY();
    math::Vec3 half = m_size * 0.5f;

    // find lowest vertex
    float lowestY = m_position.y;

    // project half-extents onto each axis and find y-components
    for (int i = 0; i < 8; i++)
    {
        // generate all 8 corner combinations
        float sx = (i & 1) ? half.x : -half.x;
        float sy = (i & 2) ? half.y : -half.y;
        float sz = (i & 4) ? half.z : -half.z;

        // vertex = center + sx*axisX + sy*axisY + sz*axisZ
        float vertexY = m_position.y + sx * m_axes[0].y + sy * m_axes[1].y + sz * m_axes[2].y;

        if (vertexY < lowestY)
        {
            lowestY = vertexY;
        }
    }

    if (lowestY < groundY)
    {
        outInfo.normal = math::Vec3{0.0f, 1.0f, 0.0f};
        outInfo.penetration = groundY - lowestY;
        return true;
    }

    return false;
}

} // namespace collision
} // namespace BulletPhysics
