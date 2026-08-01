#pragma once

#include <cmath>
#include "Vector3.hpp"

namespace Math {

    struct Quaternion {
        double w{1.0};
        double x{0.0};
        double y{0.0};
        double z{0.0};

        Quaternion() = default;
        constexpr Quaternion(double w, double x, double y, double z) 
            : w(w), x(x), y(y), z(z) {}

        [[nodiscard]] static Quaternion from_axis_angle(const Vector3& axis, double angle_rad) {
            Vector3 norm_axis = axis.normalized();
            double half_angle = angle_rad * 0.5;
            double sin_half = std::sin(half_angle);

            return Quaternion(
                std::cos(half_angle),
                norm_axis.x * sin_half,
                norm_axis.y * sin_half,
                norm_axis.z * sin_half
            );
        }

        [[nodiscard]] constexpr Quaternion conjugate() const {
            return Quaternion(w, -x, -y, -z);
        }

        [[nodiscard]] constexpr double length_squared() const {
            return w * w + x * x + y * y + z * z;
        }

        [[nodiscard]] double length() const {
            return std::sqrt(length_squared());
        }

        void normalize() {
            double len = length();
            if (len > 0.00001) {
                w /= len;
                x /= len;
                y /= len;
                z /= len;
            } else {
                w = 1.0; x = 0.0; y = 0.0; z = 0.0;
            }
        }

        [[nodiscard]] Quaternion normalized() const {
            Quaternion q = *this;
            q.normalize();
            return q;
        }

        constexpr Quaternion& operator*=(const Quaternion& rhs) {
            double new_w = w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z;
            double new_x = w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y;
            double new_y = w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x;
            double new_z = w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w;

            w = new_w; x = new_x; y = new_y; z = new_z;
            return *this;
        }

        constexpr friend Quaternion operator*(Quaternion lhs, const Quaternion& rhs) {
            lhs *= rhs;
            return lhs;
        }

        [[nodiscard]] constexpr Vector3 rotate_vector(const Vector3& v) const {
            Vector3 q_vec(x, y, z);
            Vector3 uv = q_vec.cross(v);
            Vector3 uuv = q_vec.cross(uv);

            return v + (uv * (2.0 * w)) + (uuv * 2.0);
        }
    };

} 