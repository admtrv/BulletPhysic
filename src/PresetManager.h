/*
 * PresetManager.h
 */

#pragma once

#include "Config.h"
#include "Constants.h"
#include "dynamics/forces/ForceRegistry.h"
#include "dynamics/forces/GravityForce.h"
#include "dynamics/forces/DragForce.h"
#include "dynamics/forces/AtmosphereModel.h"

#include <memory>

namespace BulletPhysic {
namespace preset {

// predefined realism levels
enum class RealismLevel {
    BASIC,          // only gravity
    SIMPLE_DRAG,    // gravity + linear drag
    REALISTIC_DRAG, // gravity + quadratic drag
    ATMOSPHERIC,    // gravity + atmospheric drag with ISA model
    CUSTOM          // user-defined combination
};

// manager for configuring physics presets
class PresetManager {
public:
    // configure forces based on realism level
    static void configure(dynamics::forces::ForceRegistry& registry, RealismLevel level, float crossSectionArea = constants::DEFAULT_SPHERE_AREA);

    // individual configuration methods
    static void configureBasic(dynamics::forces::ForceRegistry& registry);
    static void configureSimpleDrag(dynamics::forces::ForceRegistry& registry, float coefficient = constants::DEFAULT_SPHERE_LINEAR_B);
    static void configureRealisticDrag(dynamics::forces::ForceRegistry& registry, float area = constants::DEFAULT_SPHERE_AREA, float dragCoefficient = constants::DEFAULT_SPHERE_CD);
    static void configureAtmospheric(dynamics::forces::ForceRegistry& registry, float area = constants::DEFAULT_SPHERE_AREA, float dragCoefficient = constants::DEFAULT_SPHERE_CD);

    // factory methods for creating force generators
    static std::unique_ptr<dynamics::forces::GravityForce> createGravity(const math::Vec3& gravity = config::gravityVec);
    static std::unique_ptr<dynamics::forces::DragForce> createLinearDrag(float coefficient = constants::DEFAULT_SPHERE_LINEAR_B);
    static std::unique_ptr<dynamics::forces::DragForce> createQuadraticDrag(float dragCoefficient = constants::DEFAULT_SPHERE_CD, float area = constants::DEFAULT_SPHERE_AREA, float density = constants::SEA_LEVEL_DENSITY);
    static std::unique_ptr<dynamics::forces::AtmosphericDragForce> createAtmosphericDrag(float dragCoefficient = constants::DEFAULT_SPHERE_CD, float area = constants::DEFAULT_SPHERE_AREA, bool useISA = true);

};

} // namespace preset
} // namespace BulletPhysic