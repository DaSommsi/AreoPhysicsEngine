#pragma once

#include "math/Vector3.hpp"
#include "math/Quaternion.hpp"
#include <array>

namespace Math {

    struct Matrix3x3 {
        std::array<double, 9> m{
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        };

        Matrix3x3() = default;

        constexpr Matrix3x3(double m00, double m01, double m02,
                            double m10, double m11, double m12,
                            double m20, double m21, double m22)
            : m{m00, m01, m02, m10, m11, m12, m20, m21, m22} {}

        [[nodiscard]] constexpr Vector3 operator*(const Vector3& v) const {
            return Vector3{
                m[0] * v.x + m[1] * v.y + m[2] * v.z,
                m[3] * v.x + m[4] * v.y + m[5] * v.z,
                m[6] * v.x + m[7] * v.y + m[8] * v.z
            };
        }

        [[nodiscard]] constexpr Matrix3x3 operator*(const Matrix3x3& o) const {
            return Matrix3x3{
                m[0]*o.m[0] + m[1]*o.m[3] + m[2]*o.m[6],
                m[0]*o.m[1] + m[1]*o.m[4] + m[2]*o.m[7],
                m[0]*o.m[2] + m[1]*o.m[5] + m[2]*o.m[8],

                m[3]*o.m[0] + m[4]*o.m[3] + m[5]*o.m[6],
                m[3]*o.m[1] + m[4]*o.m[4] + m[5]*o.m[7],
                m[3]*o.m[2] + m[4]*o.m[5] + m[5]*o.m[8],

                m[6]*o.m[0] + m[7]*o.m[3] + m[8]*o.m[6],
                m[6]*o.m[1] + m[7]*o.m[4] + m[8]*o.m[7],
                m[6]*o.m[2] + m[7]*o.m[5] + m[8]*o.m[8]
            };
        }

        [[nodiscard]] constexpr Matrix3x3 transposed() const {
            return Matrix3x3{
                m[0], m[3], m[6],
                m[1], m[4], m[7],
                m[2], m[5], m[8]
            };
        }

        [[nodiscard]] static constexpr Matrix3x3 from_quaternion(const Quaternion& q) {
            double x = q.x, y = q.y, z = q.z, w = q.w;

            return Matrix3x3{
                1.0 - 2.0 * (y*y + z*z), 2.0 * (x*y - z*w),       2.0 * (x*z + y*w),
                2.0 * (x*y + z*w),       1.0 - 2.0 * (x*x + z*z), 2.0 * (y*z - x*w),
                2.0 * (x*z - y*w),       2.0 * (y*z + x*w),       1.0 - 2.0 * (x*x + y*y)
            };
        }
    };

}