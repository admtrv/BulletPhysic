/*
 * PresetManager.h
 */

#pragma once

#include "math/Vec3.h"
#include "Constants.h"

namespace BulletPhysic {
namespace dynamics {

// forward declaration
class PhysicsWorld;

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

// arguments structure as input
struct PresetArgs {
    // with drag
    float area = constants::DEFAULT_SPHERE_AREA;
    float cd = constants::DEFAULT_SPHERE_CD;

    // with humidity
    float humidity = 50.0f;

    // with wind
    math::Vec3 wind{0.0f, 0.0f, 0.0f};

    // with coriolis
    double latitude = 0.0;
    double longitude = 0.0;
};

class PresetManager {
public:
    // configure physics world with specified realism level
    static void configure(PhysicsWorld& world, Preset level, const PresetArgs& args = PresetArgs{});

private:
    static void configureGravityOnly(PhysicsWorld& world);
    static void configureWithDrag(PhysicsWorld& world, const PresetArgs& args);
    static void configureWithAtmosphere(PhysicsWorld& world, const PresetArgs& args);
    static void configureWithHumidity(PhysicsWorld& world, const PresetArgs& args);
    static void configureWithWind(PhysicsWorld& world, const PresetArgs& args);
    static void configureWithCoriolis(PhysicsWorld& world, const PresetArgs& args);
};

} // namespace dynamics
} // namespace BulletPhysic
