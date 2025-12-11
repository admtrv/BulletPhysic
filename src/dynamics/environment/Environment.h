/*
 * Environment.h
 */

#pragma once

#include "dynamics/PhysicsBody.h"
#include "dynamics/PhysicsContext.h"

#include <string>

namespace BulletPhysic {
namespace dynamics {
namespace environment {

// base interface for environment providers
class IEnvironment {
public:
    virtual ~IEnvironment() = default;

    // update physics context based on physics body state
    virtual void update(IPhysicsBody& body, PhysicsContext& context) = 0;

    virtual const std::string& getName() const = 0;
};

} // namespace environment
} // namespace dynamics
} // namespace BulletPhysic
