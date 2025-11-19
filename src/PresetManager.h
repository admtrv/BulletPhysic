/*
 * PresetManager.h
 */

#pragma once

#include "Constants.h"
#include "dynamics/PhysicsWorld.h"
#include "dynamics/forces/Gravity.h"
#include "dynamics/forces/Drag.h"
#include "dynamics/forces/Coriolis.h"
#include "dynamics/environment/Atmosphere.h"
#include "dynamics/environment/Humidity.h"
#include "dynamics/environment/Wind.h"
#include "dynamics/environment/Geographic.h"

namespace BulletPhysic {
namespace preset {

// realism levels: by physical effects
enum class Preset {
    GRAVITY_ONLY,       // parabolic trajectory
    WITH_DRAG,          // + air drag
    WITH_ATMOSPHERE,    // + variable density with altitude (dry air, ISA model)
    WITH_HUMIDITY,      // + humidity effects (corrects density for water vapor)
    WITH_WIND,          // + wind
    WITH_CORIOLIS,      // + Coriolis force
    CUSTOM
};

class PresetManager {
public:
    // configure physics world with specified realism level
    static void configure(dynamics::PhysicsWorld& world,
        Preset level,
        const math::Vec3& wind = math::Vec3{0.0f, 0.0f, 0.0f},
        float humidity = 50.0f,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD,
        double latitude = 0.0,
        double longitude = 0.0)
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
        case Preset::WITH_HUMIDITY:
            configureWithHumidity(world, humidity, area, cd);
            break;
        case Preset::WITH_WIND:
            configureWithWind(world, wind, humidity, area, cd);
            break;
        case Preset::WITH_CORIOLIS:
            configureWithCoriolis(world, wind, humidity, area, cd, latitude, longitude);
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

    // gravity + drag + atmosphere + humidity
    static void configureWithHumidity(dynamics::PhysicsWorld& world,
        float humidity = 50.0f,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
        world.addEnvironment(std::make_unique<dynamics::environment::Atmosphere>());
        world.addEnvironment(std::make_unique<dynamics::environment::Humidity>(humidity));
        world.addForce(std::make_unique<dynamics::forces::Drag>(cd, area));
    }

    // gravity + drag + atmosphere + humidity + wind
    static void configureWithWind(dynamics::PhysicsWorld& world,
        const math::Vec3& wind,
        float humidity = 50.0f,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
        world.addEnvironment(std::make_unique<dynamics::environment::Atmosphere>());
        world.addEnvironment(std::make_unique<dynamics::environment::Humidity>(humidity));
        world.addEnvironment(std::make_unique<dynamics::environment::Wind>(wind));
        world.addForce(std::make_unique<dynamics::forces::Drag>(cd, area));
    }

    // gravity + drag + atmosphere + humidity + wind + Coriolis
    static void configureWithCoriolis(dynamics::PhysicsWorld& world,
        const math::Vec3& wind,
        float humidity = 50.0f,
        float area = constants::DEFAULT_SPHERE_AREA,
        float cd = constants::DEFAULT_SPHERE_CD,
        double latitude = 0.0,
        double longitude = 0.0)
    {
        world.clear();
        world.addForce(std::make_unique<dynamics::forces::Gravity>());
        world.addEnvironment(std::make_unique<dynamics::environment::Atmosphere>());
        world.addEnvironment(std::make_unique<dynamics::environment::Humidity>(humidity));
        world.addEnvironment(std::make_unique<dynamics::environment::Wind>(wind));
        world.addEnvironment(std::make_unique<dynamics::environment::Geographic>(latitude, longitude));
        world.addForce(std::make_unique<dynamics::forces::Drag>(cd, area));
        world.addForce(std::make_unique<dynamics::forces::Coriolis>());
    }
};

} // namespace preset
} // namespace BulletPhysic