#pragma once
#include "math/type_def.h"

class SpatialVelocity
{
public:
    SpatialVelocity();
    SpatialVelocity(const Vec3f& v, const Vec3f& w);

    Vec3f v(const Vec3f& p);
    Vec3f w(const Vec3f& p);

    Vec3f m_w;  // angular
    Vec3f m_v;  // linear
    Vec6f m_s;
};

class SpatialForce
{
public:
    SpatialForce();
    SpatialForce(const Vec3f& v, const Vec3f& w);

    Vec3f n(const Vec3f& p);
    Vec3f f(const Vec3f& p);

    Vec3f m_n;  // angular
    Vec3f m_f;  // linear
    Vec6f m_s;
};
