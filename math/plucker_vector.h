#pragma once
#include "math/type_def.h"

class PluckerVector
{
public:
    Vec3f angular() const;
    Vec3f linear() const;
    Vec6f vector() const;

    void rotation(const Mat3f& r);
    void translation(const Mat3f t);

    Vec3f m_w;  // angular
    Vec3f m_v;  // linear
};
