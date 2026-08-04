#pragma once

#include <vector>
#include <memory>
#include "core/GameObject.hpp"
#include "physics/RigidBody3D.hpp"
#include "math/Vector3.hpp"

namespace Core{
    struct World{
        std::vector<std::unique_ptr<GameObject>> objects;
        Math::Vector3 gravity{0.0, 0.0, 0.0};

        World() = default;

        GameObject* create_object(const Physics::RigidBody3D& body, Color color){
            auto obj = std::make_unique<GameObject>(body, nullptr, color);
            GameObject* ptr = obj.get();
            objects.push_back(std::move(obj));
            return ptr;
        }  

        void update(double dt){
            for(auto& obj : objects){
                if (obj->body.inv_mass > 0.0) {
                    obj->body.add_force(gravity * obj->body.mass);
                }
            }

            for (auto& obj : objects) {
                obj->update(dt);
            }

            // Collisions
        }

        void draw() const {
            for(auto& obj : objects){
                obj->draw();
            }
        }
    };
}