#pragma once

#include <cmath>

namespace Math{

    struct Vector3 {
        double x{0.0};
        double y{0.0};
        double z{0.0};

        Vector3() = default;
        constexpr Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
        ~Vector3() = default;

        constexpr Vector3& operator+=(const Vector3& b) {
            this->x += b.x;
            this->y += b.y;
            this->z += b.z;

            return *this;
        }

        [[nodiscard]] constexpr friend Vector3 operator+(Vector3 a, const Vector3& b){
            a += b;
            return a;
        }

        constexpr Vector3& operator-=(const Vector3& b) {
            this->x -= b.x;
            this->y -= b.y;
            this->z -= b.z;

            return *this;
        }

        [[nodiscard]] constexpr friend Vector3 operator-(Vector3 a, const Vector3& b){
            a -= b;
            return a;
        }

        [[nodiscard]] constexpr Vector3 operator-() const {
            return Vector3(-x, -y, -z);
        }

        constexpr Vector3& operator*=(const double b) {
            this->x *= b;
            this->y *= b;
            this->z *= b;

            return *this;
        }

        [[nodiscard]] constexpr friend Vector3 operator*(Vector3 a, const double b){
            a *= b;
            return a;
        }

        constexpr Vector3& operator/=(const double b) {
            this->x /= b;
            this->y /= b;
            this->z /= b;

            return *this;
        }

        [[nodiscard]] constexpr friend Vector3 operator/(Vector3 a, const double b){
            a /= b;
            return a;
        }

        [[nodiscard]] constexpr double dot(const Vector3& b) const {
            return x*b.x + y*b.y + z*b.z;
        }

        [[nodiscard]] constexpr Vector3 cross(const Vector3& b) const {
            return Vector3(
                y*b.z - z*b.y,
                z*b.x - x*b.z,
                x*b.y - y*b.x
            );
        }

        [[nodiscard]] constexpr double length_squared() const {
            return x*x + y*y + z*z;
        }

        [[nodiscard]] constexpr double length() const {
            return std::sqrt(length_squared());
        }   

        [[nodiscard]] constexpr Vector3 normalized() const {
            double len = length();

            if(len > 0.00001f){
                return *this/len;
            }

            return Vector3(0.0f, 0.0f, 0.0f);
        }
    };
}