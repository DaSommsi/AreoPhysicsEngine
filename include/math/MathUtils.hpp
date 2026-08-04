#pragma once

#include "math/Vector3.hpp"
#include "math/Matrix3x3.hpp"
#include "math/Quaternion.hpp"
#include "raylib.h"
#include "raymath.h"

namespace Math::Utils
{
    [[nodiscard]] constexpr ::Vector3 Vector3ToRaylib(const Math::Vector3& b) {
        return ::Vector3{(float)b.x, (float)b.y, (float)b.z};
    }

    [[nodiscard]] constexpr Matrix QuaternionToRaylibMatrix(const Math::Quaternion& q) {
        float x = q.x, y = q.y, z = q.z, w = q.w;

        Matrix m = { 0 };
        m.m0  = 1.0f - 2.0f*(y*y + z*z);  m.m4  = 2.0f*(x*y - z*w);        m.m8  = 2.0f*(x*z + y*w);        m.m12 = 0.0f;
        m.m1  = 2.0f*(x*y + z*w);        m.m5  = 1.0f - 2.0f*(x*x + z*z);  m.m9  = 2.0f*(y*z - x*w);        m.m13 = 0.0f;
        m.m2  = 2.0f*(x*z - y*w);        m.m6  = 2.0f*(y*z + x*w);        m.m10 = 1.0f - 2.0f*(x*x + y*y);  m.m14 = 0.0f;
        m.m3  = 0.0f;                    m.m7  = 0.0f;                    m.m11 = 0.0f;                    m.m15 = 1.0f;
        return m;
    }
}
