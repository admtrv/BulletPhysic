/*
 * PresetManager.h
 */

#pragma once

#include "Constants.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Drag.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Wind.h"

namespace BulletPhysic {
namespace preset {

// realism levels: by physical effects
enum class Preset {
    GRAVITY_ONLY,       // parabolic trajectory
    WITH_DRAG,          // + air drag
    WITH_ATMOSPHERE,    // + variable density with altitude
    WITH_WIND,          // + wind
    CUSTOM
};

class PresetManager {
public:
    // configure physics world with specified realism level
    static void configure(dynamics::PhysicsWorld& world,
        Preset level,
        const math::Vec3& wind = math::Vec3{0.0f, 0.0f, 0.0f},
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD)
    {
        switch (level)
        {
        case Preset::GRAVITY_ONLY:
            configureGravityOnly(world);
            break;
        case Preset::WITH_DRAG:
            configureWithDrag(world, area, cd);
            break;
        case Preset::WITH_ATMOSPHERE:
            configureWithAtmosphere(world, area, cd);
            break;
        case Preset::WITH_WIND:
            configureWithWind(world, wind, area, cd);
            break;
        case Preset::CUSTOM:
            // user configures manually - just clear
            world.clear();
            break;
        }
    }

    // parabolic trajectory
    static void configureGravityOnly(dynamics::PhysicsWorld& world)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
    }

    // gravity + drag
    static void configureWithDrag(dynamics::PhysicsWorld& world,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
        world.addForce(std::make_unique<dynamics::forces::Drag>(cd, area));
    }

    // gravity + drag + atmosphere
    static void configureWithAtmosphere(dynamics::PhysicsWorld& world,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
        world.addEnvironment(std::make_unique<dynamics::environment::Atmosphere>());
        world.addForce(std::make_unique<dynamics::forces::Drag>(cd, area));
    }

    // gravity + drag + atmosphere + wind
    static void configureWithWind(dynamics::PhysicsWorld& world,
        const math::Vec3& wind,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
        world.addEnvironment(std::make_unique<dynamics::environment::Atmosphere>());
        world.addEnvironment(std::make_unique<dynamics::environment::Wind>(wind));
        world.addForce(std::make_unique<dynamics::forces::Drag>(cd, area));
    }
};

} // namespace preset
} // namespace BulletPhysic