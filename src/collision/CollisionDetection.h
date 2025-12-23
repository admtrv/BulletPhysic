/*
 * CollisionDetection.h
 */

#pragma once

#include "Collider.h"

#include <vector>
#include <memory>
#include <algorithm>

namespace BulletPhysics {
namespace collision {

struct Manifold {
    Collider* colliderA;
    Collider* colliderB;
    CollisionInfo info;
};

class CollisionDetection {
public:
    void addCollider(Collider* collider);
    void removeCollider(Collider* collider);
    void clear();

    void detect(std::vector<Manifold>& manifolds);

private:
    std::vector<Collider*> m_colliders;
};

} // namespace collision
} // namespace BulletPhysics
