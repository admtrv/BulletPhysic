/*
 * Integrator.h
 */

#pragma once

#include "dynamics/RigidBody.h"
#include "dynamics/PhysicsWorld.h"

namespace BulletPhysic {
namespace math {

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    virtual void step(dynamics::RigidBody& rb, dynamics::PhysicsWorld* world, float dt) = 0;
};

class EulerIntegrator final : public IIntegrator {
public:
    void step(dynamics::RigidBody& rb, dynamics::PhysicsWorld* world, float dt) override;
};

class MidpointIntegrator final : public IIntegrator {
public:
    void step(dynamics::RigidBody& rb, dynamics::PhysicsWorld* world, float dt) override;
};

class RK4Integrator final : public IIntegrator {
public:
    void step(dynamics::RigidBody& rb, dynamics::PhysicsWorld* world, float dt) override;
};

} // namespace math
} // namespace BulletPhysic
