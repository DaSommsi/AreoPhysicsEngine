#pragma once

#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "Collider.hpp"
#include "RigidBody3D.hpp"
#include <memory>

namespace Core{
    struct GameObject {
        Physics::RigidBody3D body;
        std::unique_ptr<Physics::Collider> collider{nullptr};

        Color color{GRAY};
        Model model;
        bool has_model{false};

        GameObject() = default;
        GameObject(Physics::RigidBody3D b, std::unique_ptr<Physics::Collider> col, Color c = GRAY)
            : body(b), collider(std::move(col)), color(c), has_model(false) {
            if (collider) {
                collider->rigidbody = &body;
            }
        }

        template <typename T, typename... Args>
        void set_collider(Args&&... args) {
            collider = std::make_unique<T>(&body, std::forward<Args>(args)...);
        }

        void Update(double dt){
            body.integrate(dt);
        }

        void draw(){
            rlPushMatrix();
                rlTranslatef((float)body.position.x, (float)body.position.y, (float)body.position.z);
                rlMultMatrixf(MatrixToFloatV(Math::Utils::QuaternionToRaylibMatrix(body.orientation)).v);

                DrawModel(model, Math::Utils::Vector3ToRaylib(body.position), 1.0, color);

                if (has_model) {
                    DrawModel(model, {0.0f, 0.0f, 0.0f}, 1.0f, color);
                    DrawModelWires(model, {0.0f, 0.0f, 0.0f}, 1.0f, BLACK);
                } else {
                    DrawCube({0.0f, 0.0f, 0.0f}, 2.0f, 2.0f, 2.0f, color);
                    DrawCubeWires({0.0f, 0.0f, 0.0f}, 2.0f, 2.0f, 2.0f, BLACK);
                }
            rlPopMatrix();
        }
    };
}