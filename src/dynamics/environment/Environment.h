/*
 * Environment.h
 */

#pragma once

#include "dynamics/RigidBody.h"
#include "dynamics/PhysicsContext.h"

#include <string>

namespace BulletPhysic {
namespace dynamics {
namespace environment {

// base interface for environment providers
class IEnvironment {
public:
    virtual ~IEnvironment() = default;

    // update physics context based on rigid body state
    virtual void update(PhysicsContext& context, const RigidBody& rb) = 0;

    virtual const std::string& getName() const = 0;
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
