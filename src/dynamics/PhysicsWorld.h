/*
 * PhysicsWorld.h
 */

#pragma once

#include "PhysicsContext.h"
#include "RigidBody.h"
#include "forces/Force.h"
#include "environment/Environment.h"

#include <vector>
#include <memory>
#include <string>

namespace BulletPhysic {
namespace dynamics {

// physics world manages forces and environment
class PhysicsWorld {
public:
    PhysicsWorld() = default;
    ~PhysicsWorld() = default;

    PhysicsWorld(const PhysicsWorld&) = delete;
    PhysicsWorld& operator=(const PhysicsWorld&) = delete;

    PhysicsWorld(PhysicsWorld&&) = default;
    PhysicsWorld& operator=(PhysicsWorld&&) = default;

    // add force or environment
    void addForce(std::unique_ptr<IForce> force);
    void addEnvironment(std::unique_ptr<environment::IEnvironment> environment);

    // remove all
    void clear();

    // apply all forces to rigid body
    void applyForces(RigidBody& rb, float dt);

    // get by name
    IForce* getForce(const std::string& name);
    environment::IEnvironment* getEnvironment(const std::string& name);

    // counts
    size_t forceCount() const { return m_forces.size(); }
    size_t environmentCount() const { return m_environments.size(); }

private:
    std::vector<std::unique_ptr<IForce>> m_forces;
    std::vector<std::unique_ptr<environment::IEnvironment>> m_environments;
    PhysicsContext m_context;
};

} // namespace dynamics
} // namespace BulletPhysic
