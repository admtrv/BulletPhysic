/*
 * Integrator.h
 */

#pragma once

#include "Config.h"
#include "dynamics/RigidBody.h"
#include "dynamics/forces/ForceRegistry.h"

namespace BulletPhysic {
namespace math {

class IIntegrator {
public:
    virtual ~IIntegrator() = default;
    virtual void step(dynamics::RigidBody& rb, dynamics::forces::ForceRegistry* forceRegistry, float dt) = 0;
};

class EulerIntegrator final : public IIntegrator {
public:
    void step(dynamics::RigidBody& rb, dynamics::forces::ForceRegistry* forceRegistry, float dt) override;
};

class RK4Integrator final : public IIntegrator {
public:
    void step(dynamics::RigidBody& rb, dynamics::forces::ForceRegistry* forceRegistry, float dt) override;
};

} // namespace math
} // namespace BulletPhysic
