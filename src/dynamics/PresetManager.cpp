/*
 * PresetManager.cpp
 */

#include "PresetManager.h"
#include "PhysicsWorld.h"
#include "forces/Gravity.h"
#include "forces/drag/Drag.h"
#include "forces/Coriolis.h"
#include "environment/Atmosphere.h"
#include "environment/Humidity.h"
#include "environment/Wind.h"
#include "environment/Geographic.h"

namespace BulletPhysic {
namespace dynamics {

void PresetManager::configure(PhysicsWorld& world, Preset level, const PresetArgs& args)
{
    switch (level)
    {
    case Preset::GRAVITY_ONLY:
        configureGravityOnly(world);
        break;
    case Preset::WITH_DRAG:
        configureWithDrag(world, args);
        break;
    case Preset::WITH_ATMOSPHERE:
        configureWithAtmosphere(world, args);
        break;
    case Preset::WITH_HUMIDITY:
        configureWithHumidity(world, args);
        break;
    case Preset::WITH_WIND:
        configureWithWind(world, args);
        break;
    case Preset::WITH_CORIOLIS:
        configureWithCoriolis(world, args);
        break;
    case Preset::CUSTOM:
        // user configures manually - just clear
        world.clear();
        break;
    }
}

void PresetManager::configureGravityOnly(PhysicsWorld& world)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
}

void PresetManager::configureWithDrag(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addForce(std::make_unique<forces::drag::Drag>());
}

void PresetManager::configureWithAtmosphere(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addForce(std::make_unique<forces::drag::Drag>());
}

void PresetManager::configureWithHumidity(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addEnvironment(std::make_unique<environment::Humidity>(args.humidity));
    world.addForce(std::make_unique<forces::drag::Drag>());
}

void PresetManager::configureWithWind(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addEnvironment(std::make_unique<environment::Humidity>(args.humidity));
    world.addEnvironment(std::make_unique<environment::Wind>(args.wind));
    world.addForce(std::make_unique<forces::drag::Drag>());
}

void PresetManager::configureWithCoriolis(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addEnvironment(std::make_unique<environment::Humidity>(args.humidity));
    world.addEnvironment(std::make_unique<environment::Wind>(args.wind));
    world.addEnvironment(std::make_unique<environment::Geographic>(args.latitude, args.longitude));
    world.addForce(std::make_unique<forces::drag::Drag>());
    world.addForce(std::make_unique<forces::Coriolis>());
}

} // namespace dynamics
} // namespace BulletPhysic
