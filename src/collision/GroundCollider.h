/*
 * GroundCollider.h
 */

#pragma once

#include "Collider.h"
#include "BoxCollider.h"

namespace BulletPhysic {
namespace collision {

class BoxCollider;

// represents an infinite ground plane (similar to WorldBoundaryShape2D in Godot)
class GroundCollider : public Collider {
public:
    explicit GroundCollider(float groundY = 0.0f);

    CollisionShape getShape() const override { return CollisionShape::Ground; }
    const math::Vec3& getPosition() const override { return m_position; }
    void setPosition(const math::Vec3& pos) override;

    float getGroundY() const { return m_position.y; }
    void setGroundY(float level);

    bool testCollision(const Collider& other, CollisionInfo& outInfo) const override;
    bool testPoint(const math::Vec3& point) const override;

    bool testCollisionWithBox(const BoxCollider& box, CollisionInfo& outInfo) const;

private:
    math::Vec3 m_position{};

    friend class BoxCollider;
};

} // namespace collision
} // namespace BulletPhysic
