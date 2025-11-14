/*
 * ForceRegistry.h
 */
#pragma once

#include "Force.h"

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

    // add/remove force
    void add(std::unique_ptr<IForce> force);
    bool remove(size_t index);
    void clear();

    // manage rigid body
    void applyForces(RigidBody& rb, float dt); // apply all active forces
    void clearAccumulators(RigidBody& rb); // clear all accumulated forces

    // get force by index
    IForce* get(size_t index);
    const IForce* get(size_t index) const;

    // get force by name
    IForce* getByName(const std::string& forceName);
    const IForce* getByName(const std::string& forceName) const;

    size_t count() const;

private:
    std::vector<std::unique_ptr<IForce>> m_forces;
};

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic