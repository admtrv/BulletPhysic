/*
 * ForceRegistry.cpp
 */

#include "ForceRegistry.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {

void ForceRegistry::add(std::unique_ptr<IForceGenerator> generator)
{
    if (generator)
    {
        m_generators.push_back(std::move(generator));
    }
}

bool ForceRegistry::remove(size_t index)
{
    if (index < m_generators.size())
    {
        m_generators.erase(m_generators.begin() + index);
        return true;
    }

    return false;
}

void ForceRegistry::clear()
{
    m_generators.clear();
}

void ForceRegistry::applyForces(RigidBody& rb, float dt)
{
    for (auto& generator : m_generators)
    {
        if (generator && generator->isActive())
        {
            generator->apply(rb, dt);
        }
    }
}

void ForceRegistry::clearAccumulators(RigidBody& rb)
{
    rb.clearForces();
}

IForceGenerator* ForceRegistry::get(size_t index)
{
    if (index < m_generators.size())
    {
        return m_generators[index].get();
    }
    return nullptr;
}

const IForceGenerator* ForceRegistry::get(size_t index) const
{
    if (index < m_generators.size())
    {
        return m_generators[index].get();
    }
    return nullptr;
}

size_t ForceRegistry::count() const
{
    return m_generators.size();
}

} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic