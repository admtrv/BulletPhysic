/*
 * DragModel.h
 */

#pragma once

#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "Constants.h"
#include "math/Algorithms.h"

namespace BulletPhysic {
namespace dynamics {
namespace forces {
namespace drag {

// drag curve data file paths
inline const std::string DRAG_CURVE_G1 = "../BulletPhysic/assets/data/g1.txt";
inline const std::string DRAG_CURVE_G2 = "../BulletPhysic/assets/data/g2.txt";
inline const std::string DRAG_CURVE_G5 = "../BulletPhysic/assets/data/g5.txt";
inline const std::string DRAG_CURVE_G6 = "../BulletPhysic/assets/data/g6.txt";
inline const std::string DRAG_CURVE_G7 = "../BulletPhysic/assets/data/g7.txt";
inline const std::string DRAG_CURVE_G8 = "../BulletPhysic/assets/data/g8.txt";
inline const std::string DRAG_CURVE_GL = "../BulletPhysic/assets/data/gl.txt";

enum class DragCurveModel {
    G1,
    G2,
    G5,
    G6,
    G7,
    G8,
    GL,
    CUSTOM
};

// represents a single drag curve from table
class DragCurve {
public:
    // load curve from file (mach, cd pairs)
    bool loadFromFile(const std::string& filename);

    // get Cd for given mach number (linear interpolation)
    float getCd(float mach) const;

    // get closest Cd to exact mach point
    float getCdNearest(float mach) const;

private:
    std::vector<float> m_machNumbers;
    std::vector<float> m_dragCoefficients;

    int findClosestIndex(float mach) const;
};

// interface for drag model selection
class IDragModel {
public:
    virtual ~IDragModel() = default;
    virtual float getCd(float mach) const = 0;
};

// standard G1-G8, GL curves
class StandardDragModel : public IDragModel {
public:
    explicit StandardDragModel(DragCurveModel model);

    float getCd(float mach) const override;
    DragCurveModel getModel() const { return m_model; }

private:
    DragCurveModel m_model;
    DragCurve m_curve;

    std::string getModelFilename(DragCurveModel model) const;
};

// custom constant Cd
class CustomDragModel : public IDragModel {
public:
    explicit CustomDragModel(float cd) : m_cd(cd) {}

    float getCd(float mach) const override { return m_cd; }
    float getCustomCd() const { return m_cd; }

private:
    float m_cd;
};

} // namespace drag
} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
