#pragma once

#include "math/Vector3.hpp"
#include "math/Quaternion.hpp"
#include "math/Matrix3x3.hpp"
#include "physics/RigidBody3d.hpp"
#include "math/MathUtils.hpp"
#include <vector>
#include <array>

namespace Physics {
    
    struct Collider {
        
        RigidBody3D* rigidbody{nullptr};
        Math::Vector3 local_offset{0.0, 0.0, 0.0};

        Collider() = default;
        Collider(RigidBody3D* r, Math::Vector3 offset = {0.0, 0.0, 0.0}) 
            : rigidbody(r), local_offset(offset) {}
        virtual ~Collider() = default;

        Collider(const Collider&) = default;
        Collider& operator=(const Collider&) = default;
        Collider(Collider&&) noexcept = default;
        Collider& operator=(Collider&&) noexcept = default;

        virtual std::vector<Math::Vector3> get_world_points() const = 0;
    };

    struct BoxCollider : public Collider {
        Math::Vector3 size{1.0, 1.0, 1.0};

        BoxCollider() = default;
        BoxCollider(RigidBody3D* r, Math::Vector3 size, Math::Vector3 offset = {0.0, 0.0, 0.0})
            : Collider(r, offset), size(size) {}

        [[nodiscard]] std::array<Math::Vector3, 8> get_local_points() const {
            Math::Vector3 h = size * 0.5;
            std::array<Math::Vector3, 8> p_local;

            for (int i = 0; i < 8; ++i) {
                p_local[i] = Math::Vector3{
                    (i & 1) ? h.x : -h.x,
                    (i & 2) ? h.y : -h.y,
                    (i & 4) ? h.z : -h.z
                };
            }
            return p_local;
        }

        [[nodiscard]] std::vector<Math::Vector3> get_world_points() const override {
            std::vector<Math::Vector3> p_world;
            p_world.reserve(8);

            if (!rigidbody) return p_world;

            Math::Matrix3x3 R = Math::Matrix3x3::from_quaternion(rigidbody->orientation);
            auto p_local = get_local_points();

            for (const Math::Vector3& p : p_local) {
                p_world.push_back(rigidbody->position + R * (p + local_offset));
            }

            return p_world;
        }
    };
}