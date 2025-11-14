/*
 * PresetManager.cpp
 */

#include "PresetManager.h"

namespace BulletPhysic {
namespace preset {

void PresetManager::configure(dynamics::forces::ForceRegistry& registry, RealismLevel level, float projectileArea)
{
    registry.clear();

    switch (level) {
        case RealismLevel::BASIC:
            configureBasic(registry);
            break;
        case RealismLevel::SIMPLE_DRAG:
            configureSimpleDrag(registry);
            break;
        case RealismLevel::REALISTIC_DRAG:
            configureRealisticDrag(registry, projectileArea);
            break;
        case RealismLevel::ATMOSPHERIC:
            configureAtmospheric(registry, projectileArea);
            break;
        case RealismLevel::WIND:
            configureWind(registry, math::Vec3{0.0f, 0.0f, 2.0f}, projectileArea);
            break;
        case RealismLevel::CUSTOM:
            // user will add forces manually
            break;
    }
}

void PresetManager::configureBasic(dynamics::forces::ForceRegistry& registry)
{
    // just gravity
    registry.add(std::make_unique<dynamics::forces::GravityForce>());
}

void PresetManager::configureSimpleDrag(dynamics::forces::ForceRegistry& registry, float coefficient)
{
    // gravity
    registry.add(std::make_unique<dynamics::forces::GravityForce>());

    // simple linear drag
    auto dragForce = std::make_unique<dynamics::forces::DragForce>(std::make_unique<dynamics::forces::LinearDrag>(coefficient));
    registry.add(std::move(dragForce));
}

void PresetManager::configureRealisticDrag(dynamics::forces::ForceRegistry& registry, float area, float dragCoefficient)
{
    // gravity
    registry.add(std::make_unique<dynamics::forces::GravityForce>());

    // quadratic drag with fixed air density
    auto dragModel = std::make_unique<dynamics::forces::QuadraticDrag>(dragCoefficient);
    auto dragForce = std::make_unique<dynamics::forces::DragForce>(std::move(dragModel), area, constants::SEA_LEVEL_DENSITY);

    registry.add(std::move(dragForce));
}

void PresetManager::configureAtmospheric(dynamics::forces::ForceRegistry& registry, float area, float dragCoefficient)
{
    // gravity
    registry.add(std::make_unique<dynamics::forces::GravityForce>());

    // atmospheric drag with ISA model
    auto dragModel = std::make_unique<dynamics::forces::QuadraticDrag>(dragCoefficient);
    auto atmModel = std::make_unique<dynamics::forces::ISAModel>();
    auto atmDragForce = std::make_unique<dynamics::forces::AtmosphericDragForce>(std::move(dragModel), std::move(atmModel), area);
    registry.add(std::move(atmDragForce));
}

void PresetManager::configureWind(dynamics::forces::ForceRegistry& registry, const math::Vec3& windVelocity, float area, float dragCoefficient)
{
    // gravity
    registry.add(std::make_unique<dynamics::forces::GravityForce>());

    // wind drag with ISA model
    auto windDragModel = std::make_unique<dynamics::forces::QuadraticDrag>(dragCoefficient);
    auto windAtmModel = std::make_unique<dynamics::forces::ISAModel>();
    auto windDragForce = std::make_unique<dynamics::forces::WindDragForce>(windVelocity, std::move(windDragModel), area, constants::SEA_LEVEL_DENSITY, std::move(windAtmModel));
    registry.add(std::move(windDragForce));
}

std::unique_ptr<dynamics::forces::GravityForce> PresetManager::createGravity(const math::Vec3& gravity)
{
    return std::make_unique<dynamics::forces::GravityForce>(gravity);
}

std::unique_ptr<dynamics::forces::DragForce> PresetManager::createLinearDrag(float coefficient)
{
    return std::make_unique<dynamics::forces::DragForce>(std::make_unique<dynamics::forces::LinearDrag>(coefficient));
}

std::unique_ptr<dynamics::forces::DragForce> PresetManager::createQuadraticDrag(float dragCoefficient, float area, float density)
{
    return std::make_unique<dynamics::forces::DragForce>(std::make_unique<dynamics::forces::QuadraticDrag>(dragCoefficient), area, density);
}

std::unique_ptr<dynamics::forces::AtmosphericDragForce> PresetManager::createAtmosphericDrag(float dragCoefficient, float area, bool useISA)
{
    auto dragModel = std::make_unique<dynamics::forces::QuadraticDrag>(dragCoefficient);

    std::unique_ptr<dynamics::forces::IAtmosphereModel> atmModel;
    if (useISA)
    {
        atmModel = std::make_unique<dynamics::forces::ISAModel>();
    }
    else
    {
        atmModel = std::make_unique<dynamics::forces::ExponentialAtmosphere>();
    }

    auto atmDrag = std::make_unique<dynamics::forces::AtmosphericDragForce>(std::move(dragModel),std::move(atmModel), area);

    return atmDrag;
}

std::unique_ptr<dynamics::forces::WindDragForce> PresetManager::createWind(const math::Vec3& windVelocity)
{
    auto dragModel = std::make_unique<dynamics::forces::QuadraticDrag>();
    auto atmModel = std::make_unique<dynamics::forces::ISAModel>();
    return std::make_unique<dynamics::forces::WindDragForce>(
        windVelocity,
        std::move(dragModel),
        constants::DEFAULT_SPHERE_AREA,
        constants::SEA_LEVEL_DENSITY,
        std::move(atmModel)
    );
}

} // namespace preset
} // namespace BulletPhysic