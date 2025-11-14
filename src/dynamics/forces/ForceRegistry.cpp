/*
 * ForceRegistry.cpp
 */

#include "ForceRegistry.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {

void ForceRegistry::add(std::unique_ptr<IForce> force)
{
    if (force)
    {
        m_forces.push_back(std::move(force));
    }
}

bool ForceRegistry::remove(size_t index)
{
    if (index < m_forces.size())
    {
        m_forces.erase(m_forces.begin() + index);
        return true;
    }

    return false;
}

void ForceRegistry::clear()
{
    m_forces.clear();
}

void ForceRegistry::applyForces(RigidBody& rb, float dt)
{
    for (auto& force : m_forces)
    {
        if (force && force->isActive())
        {
            force->apply(rb, dt);
        }
    }
}

void ForceRegistry::clearAccumulators(RigidBody& rb)
{
    rb.clearForces();
}

IForce* ForceRegistry::get(size_t index)
{
    if (index < m_forces.size())
    {
        return m_forces[index].get();
    }
    return nullptr;
}

const IForce* ForceRegistry::get(size_t index) const
{
    if (index < m_forces.size())
    {
        return m_forces[index].get();
    }
    return nullptr;
}

IForce* ForceRegistry::getByName(const std::string& forceName)
{
    for (auto& force : m_forces)
    {
        if (force && force->getName() == forceName)
        {
            return force.get();
        }
    }
    return nullptr;
}

const IForce* ForceRegistry::getByName(const std::string& forceName) const
{
    for (const auto& force : m_forces)
    {
        if (force && force->getName() == forceName)
        {
            return force.get();
        }
    }
    return nullptr;
}

size_t ForceRegistry::count() const
{
    return m_forces.size();
}

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic