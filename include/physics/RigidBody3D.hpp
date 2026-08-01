#pragma once

#include <iostream>
#include "Vector3.hpp"
#include "Quaternion.hpp"
#include "Matrix3x3.hpp"

namespace Physics {
    
    struct RigidBody3D {
        Math::Vector3 position{0.0, 0.0, 0.0};
        Math::Vector3 linear_velocity{0.0, 0.0, 0.0};
        Math::Quaternion orientation{1.0, 0.0, 0.0, 0.0};
        Math::Vector3 angular_velocity{0.0, 0.0, 0.0};

        double mass{1.0};
        double inv_mass{1.0};
        
        Math::Matrix3x3 inv_inertia_tensor_body;
        Math::Matrix3x3 inv_inertia_tensor_world;

        Math::Vector3 force_accumulator{0.0, 0.0, 0.0};
        Math::Vector3 torque_accumulator{0.0, 0.0, 0.0};

        RigidBody3D() = default;

        RigidBody3D(Math::Vector3 position, Math::Quaternion orientation, double mass) 
            : position(position), orientation(orientation) {
            set_mass(mass);
            set_box_inertia(1.0, 1.0, 1.0);
        }

        void set_mass(double new_mass) {
            mass = new_mass;
            inv_mass = (mass > 0.0) ? (1.0 / mass) : 0.0;
        }

        void set_box_inertia(double width, double height, double depth) {
            if (inv_mass == 0.0) {
                inv_inertia_tensor_body = Math::Matrix3x3{0,0,0, 0,0,0, 0,0,0};
                return;
            }

            double ix = (1.0 / 12.0) * mass * (height * height + depth * depth);
            double iy = (1.0 / 12.0) * mass * (width * width + depth * depth);
            double iz = (1.0 / 12.0) * mass * (width * width + height * height);

            inv_inertia_tensor_body = Math::Matrix3x3{
                1.0 / ix, 0.0,      0.0,
                0.0,      1.0 / iy, 0.0,
                0.0,      0.0,      1.0 / iz
            };
        }

        void add_force(const Math::Vector3& force){
            force_accumulator += force;
        }

        void add_force_at_point(const Math::Vector3& force, const Math::Vector3& point){
            force_accumulator += force;
            torque_accumulator += Math::Vector3{point - position}.cross(force);
        }

        void clear_forces(){
            force_accumulator = {0.0, 0.0, 0.0};
            torque_accumulator = {0.0, 0.0, 0.0};
        }

        void integrate(double dt) {
            if (inv_mass == 0.0) return;

            // Linear dynamics
            Math::Vector3 a = force_accumulator * inv_mass;
            linear_velocity += a * dt;
            position += linear_velocity * dt;

            // Transform inverse inertia tensor to world space: I_world^-1 = R * I_body^-1 * R^T
            Math::Matrix3x3 R = Math::Matrix3x3::from_quaternion(orientation);
            inv_inertia_tensor_world = R * inv_inertia_tensor_body * R.transposed();

            // Angular dynamics
            Math::Vector3 alpha = inv_inertia_tensor_world * torque_accumulator;
            angular_velocity += alpha * dt;

            // Quaternion orientation update
            orientation = orientation + 0.5 * Math::Quaternion(0.0, angular_velocity.x, angular_velocity.y, angular_velocity.z) * orientation * dt;
            orientation.normalize();

            clear_forces();
        }
    };
}