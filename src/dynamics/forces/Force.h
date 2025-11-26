/*
 * Force.h
 */

#pragma once

#include "dynamics/RigidBody.h"
#include "dynamics/PhysicsContext.h"
#include "math/Vec3.h"

#include <string>

namespace BulletPhysic {
namespace dynamics {

// base interface for forces
class IForce {
public:
    virtual ~IForce() = default;

    // apply force to rigid body using context
    virtual void apply(RigidBody& rb, PhysicsContext& context, float dt) = 0;

    // check if this force should be active
    virtual bool isActive() const { return true; }

    // get force name for identification
    virtual const std::string& getName() const = 0;
};

} // namespace dynamics
} // namespace BulletPhysic
