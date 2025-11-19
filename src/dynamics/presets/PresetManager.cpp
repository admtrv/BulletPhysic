/*
 * PresetManager.cpp
 */

#include "PresetManager.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Drag.h"
#include "dynamics/forces/Coriolis.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Humidity.h"
#include "dynamics/environment/Wind.h"
#include "dynamics/environment/Geographic.h"

namespace BulletPhysic {
namespace dynamics {
namespace presets {

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
    world.addForce(std::make_unique<forces::Drag>(args.cd, args.area));
}

void PresetManager::configureWithAtmosphere(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addForce(std::make_unique<forces::Drag>(args.cd, args.area));
}

void PresetManager::configureWithHumidity(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addEnvironment(std::make_unique<environment::Humidity>(args.humidity));
    world.addForce(std::make_unique<forces::Drag>(args.cd, args.area));
}

void PresetManager::configureWithWind(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addEnvironment(std::make_unique<environment::Humidity>(args.humidity));
    world.addEnvironment(std::make_unique<environment::Wind>(args.wind));
    world.addForce(std::make_unique<forces::Drag>(args.cd, args.area));
}

void PresetManager::configureWithCoriolis(PhysicsWorld& world, const PresetArgs& args)
{
    world.clear();
    world.addForce(std::make_unique<forces::Gravity>());
    world.addEnvironment(std::make_unique<environment::Atmosphere>());
    world.addEnvironment(std::make_unique<environment::Humidity>(args.humidity));
    world.addEnvironment(std::make_unique<environment::Wind>(args.wind));
    world.addEnvironment(std::make_unique<environment::Geographic>(args.latitude, args.longitude));
    world.addForce(std::make_unique<forces::Drag>(args.cd, args.area));
    world.addForce(std::make_unique<forces::Coriolis>());
}

} // namespace presets
} // namespace dynamics
} // namespace BulletPhysic
