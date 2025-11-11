/*
 * ForceGenerator.h
 */

#pragma once

#include "dynamics/RigidBody.h"
#include "math/Vec3.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {

class IForceGenerator {
public:
    virtual ~IForceGenerator() = default;

    // apply force to rigid body
    // dt passed for forces that need time-based calculations
    virtual void apply(RigidBody& rb, float dt) = 0;

    // check if this generator should be active
    virtual bool isActive() const { return true; }
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic