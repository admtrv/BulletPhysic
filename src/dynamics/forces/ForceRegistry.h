/*
 * ForceRegistry.h
 */
#pragma once

#include "ForceGenerator.h"

#include <vector>
#include <memory>
#include <string>

namespace BulletPhysic {
namespace dynamics {
namespace forces {

// registry for managing and applying multiple forces to rigid bodies
class ForceRegistry {
public:
    ForceRegistry() = default;
    ~ForceRegistry() = default;

    ForceRegistry(const ForceRegistry&) = delete;
    ForceRegistry& operator=(const ForceRegistry&) = delete;

    ForceRegistry(ForceRegistry&&) = default;
    ForceRegistry& operator=(ForceRegistry&&) = default;

    // add/remove force generator
    void add(std::unique_ptr<IForceGenerator> generator);
    bool remove(size_t index);
    void clear();

    // manage rigid body
    void applyForces(RigidBody& rb, float dt); // apply all active forces
    void clearAccumulators(RigidBody& rb); // clear all accumulated forces

    // get force generator
    IForceGenerator* get(size_t index);
    const IForceGenerator* get(size_t index) const;

    size_t count() const;

private:
    std::vector<std::unique_ptr<IForceGenerator>> m_generators;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic