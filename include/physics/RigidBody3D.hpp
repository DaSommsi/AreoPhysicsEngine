#pragma once

#include <iostream>
#include "Vector3.hpp"
#include "Quaternion.hpp"

namespace Physics {
    
    struct RigidBody3D {
        Math::Vector3 position{0.0, 0.0, 0.0};
        Math::Vector3 linear_velocity{0.0, 0.0, 0.0};
        Math::Quaternion orientation{1.0, 0.0, 0.0, 0.0};
        Math::Vector3 angular_velocity{0.0, 0.0, 0.0};

        double mass{1.0};
        double inertia_tensor{1.0};

        Math::Vector3 force_accumulator{0.0, 0.0, 0.0};
        Math::Vector3 torque_accumulator{0.0, 0.0, 0.0};

        RigidBody3D() = default;
        RigidBody3D(Math::Vector3 position, Math::Quaternion orientation, double mass, double inertia_tensor) : position(position), orientation(orientation), mass(mass), inertia_tensor(inertia_tensor) {}
        ~RigidBody3D() = default;

        void add_force(const Math::Vector3& force){
            force_accumulator += force;
            std::cout << force_accumulator.x << " | " << force_accumulator.y << " | " << force_accumulator.z << "\n";
        }

        void add_force_at_point(const Math::Vector3& force, const Math::Vector3& point){
            force_accumulator += force;
            torque_accumulator += Math::Vector3{point - position}.cross(force);
        }

        void clear_forces(){
            force_accumulator = {0.0, 0.0, 0.0};
            torque_accumulator = {0.0, 0.0, 0.0};
        }

        void integrate(double dt){
            auto a = force_accumulator / mass;
            linear_velocity += a*dt;
            position += linear_velocity*dt;

            auto alpha = torque_accumulator / inertia_tensor;
            angular_velocity += alpha*dt;
            orientation = orientation + 0.5 * Math::Quaternion(0.0f, angular_velocity.x, angular_velocity.y, angular_velocity.z) * orientation * dt;
            orientation.normalize();

            clear_forces();
        }
    };
}