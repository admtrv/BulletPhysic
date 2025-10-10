/*
 * Angles.h
 */

#pragma once

#include "Constants.h"

#include <numbers>

namespace luchphysic {
namespace math {

inline float deg2rad(float deg)
{
    return deg * (Pi / 180.0f);
}
inline float rad2deg(float rad)
{
    return rad * (180.0f / Pi);
}

} // namespace math
} // namespace luchphysic
