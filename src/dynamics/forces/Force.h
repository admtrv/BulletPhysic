/*
 * Force.h
 */

#pragma once

#include "dynamics/RigidBody.h"
#include "math/Vec3.h"

#include <string>

namespace BulletPhysic {
namespace dynamics {
namespace forces {

class IForce {
public:
    virtual ~IForce() = default;

    // apply force to rigid body
    // dt passed for forces that need time-based calculations
    virtual void apply(RigidBody& rb, float dt) = 0;

    // check if this force should be active
    virtual bool isActive() const { return true; }

    // get force name for identification
    virtual const std::string& getName() const = 0;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
