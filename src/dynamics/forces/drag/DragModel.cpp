/*
 * DragModel.cpp
 */

#include "DragModel.h"
#include <iostream>

namespace BulletPhysic {
namespace dynamics {
namespace forces {
namespace drag {

bool DragCurve::loadFromFile(const std::string& filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        return false;
    }

    m_machNumbers.clear();
    m_dragCoefficients.clear();

    std::string line;
    while (std::getline(file, line))
    {
        // skip empty lines
        if (line.empty() || line[0] == '#')
        {
            continue;
        }

        std::istringstream iss(line);

        float mach;
        float cd;

        if (iss >> mach >> cd)
        {
            m_machNumbers.push_back(mach);
            m_dragCoefficients.push_back(cd);
        }
    }

    return !m_machNumbers.empty();
}

int DragCurve::findClosestIndex(float mach) const
{
    if (m_machNumbers.empty())
    {
        return 0;
    }

    int idx = 0;
    float minDiff = std::abs(m_machNumbers[0] - mach);

    for (size_t i = 1; i < m_machNumbers.size(); i++)
    {
        float diff = std::abs(m_machNumbers[i] - mach);
        if (diff < minDiff)
        {
            minDiff = diff;
            idx = i;
        }
    }

    return idx;
}

float DragCurve::getCd(float mach) const
{
    if (m_machNumbers.empty())
    {
        return constants::DEFAULT_SPHERE_CD;  // default fallback
    }

    // clamp to range
    if (mach <= m_machNumbers.front())
    {
        return m_dragCoefficients.front();
    }
    if (mach >= m_machNumbers.back())
    {
        return m_dragCoefficients.back();
    }

    // find two surrounding points for linear interpolation
    size_t i = 0;
    while (i < m_machNumbers.size() - 1 && m_machNumbers[i + 1] < mach)
    {
        ++i;
    }

    float mach1 = m_machNumbers[i];
    float mach2 = m_machNumbers[i + 1];

    float cd1 = m_dragCoefficients[i];
    float cd2 = m_dragCoefficients[i + 1];

    // linear interpolation
    float t = (mach - mach1) / (mach2 - mach1);
    return math::lerp(cd1, cd2, t);
}

float DragCurve::getCdNearest(float mach) const
{
    int idx = findClosestIndex(mach);
    return m_dragCoefficients[idx];
}

std::string StandardDragModel::getModelFilename(DragCurveModel model) const
{
    switch (model)
    {
    case DragCurveModel::G1:
        return DRAG_CURVE_G1;
    case DragCurveModel::G2:
        return DRAG_CURVE_G2;
    case DragCurveModel::G5:
        return DRAG_CURVE_G5;
    case DragCurveModel::G6:
        return DRAG_CURVE_G6;
    case DragCurveModel::G7:
        return DRAG_CURVE_G7;
    case DragCurveModel::G8:
        return DRAG_CURVE_G8;
    case DragCurveModel::GL:
        return DRAG_CURVE_GL;
    default:
        return "";
    }
}

StandardDragModel::StandardDragModel(DragCurveModel model) : m_model(model) {
    std::string filename = getModelFilename(model);

    bool loaded = m_curve.loadFromFile(filename);
    if (!loaded)
    {
        std::cerr << "failed to load curve from: " << filename << std::endl;
    }
}

float StandardDragModel::getCd(float mach) const
{
    return m_curve.getCd(mach);
}

} // namespace drag
} // namespace forces
} // namespace dynamics
} // namespace BulletPhysic
