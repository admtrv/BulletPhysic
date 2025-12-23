/*
 * Collider.h
 */

#pragma once

#include "math/Vec3.h"

namespace BulletPhysics {
namespace collision {

enum class CollisionShape {
    Box,
    Ground,
};

struct CollisionInfo {
    float penetration = 0.0f;
    math::Vec3 normal{};
};

class Collider {
public:
    virtual ~Collider() = default;

    virtual CollisionShape getShape() const = 0;
    virtual const math::Vec3& getPosition() const = 0;
    virtual void setPosition(const math::Vec3& pos) = 0;

    virtual bool testCollision(const Collider& other, CollisionInfo& outInfo) const = 0;
    virtual bool testPoint(const math::Vec3& point) const = 0;
};

} // namespace collision
} // namespace BulletPhysics
