#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include <physics/Vector3.hpp>
#include <physics/Quaternion.hpp>
#include <physics/RigidBody3D.hpp>

[[nodiscard]] constexpr Vector3 ToRaylib(const Math::Vector3& b) {
    return Vector3{(float)b.x, (float)b.y, (float)b.z};
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

void DrawVector3D(Vector3 start, Vector3 vector, Color color) {
    // Das Ende des Vektors ist: Startpunkt + Vektor
    Vector3 end = { 
        start.x + vector.x, 
        start.y + vector.y, 
        start.z + vector.z 
    };

    // 1. Schaft des Pfeils zeichnen
    DrawLine3D(start, end, color);

    // 2. Spitze zeichnen (eine kleine Kugel am Ende)
    DrawSphere(end, 0.15f, color);
}

void UpdateCameraCustom(Camera3D& camera){
    Vector3 rotation = { 0.0f, 0.0f, 0.0f };

    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
        DisableCursor();
    }
    
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
        Vector2 mouseDelta = GetMouseDelta();
        float sensitivity = 0.05f;

        rotation.x = mouseDelta.x * sensitivity;
        rotation.y = mouseDelta.y * sensitivity;
        rotation.z = 0.0f;
    }

    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
        EnableCursor();
    }

    Vector3 movement = { 0.0f, 0.0f, 0.0f };
    float speed = 0.2f;

    if (IsKeyDown(KEY_W)) movement.x += speed; // Vorwärts
    if (IsKeyDown(KEY_S)) movement.x -= speed; // Rückwärts
    if (IsKeyDown(KEY_D)) movement.y += speed; // Rechts
    if (IsKeyDown(KEY_A)) movement.y -= speed; // Links
    if (IsKeyDown(KEY_E)) movement.z += speed; // Sauber nach OBEN fliegen
    if (IsKeyDown(KEY_Q)) movement.z -= speed; // Sauber nach UNTEN fliegen

    UpdateCameraPro(&camera, movement, rotation, 0.0f);
}

int main(void){
    InitWindow(1920,1080, "PhysicsEngine");
    SetTargetFPS(60);

    Camera3D camera = {0};
    camera.position = {0.0f, 10.0f, 10.0f};
    camera.target   = {0.0f, 0.0f, 0.0f};
    camera.up       = {0.0f, 1.0f, 0.0f};
    camera.fovy     = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    Physics::RigidBody3D r({0.0, 0.0, 0.0}, {1.0, 0.0, 0.0, 0.0}, 1.0, 1.0);

    while(!WindowShouldClose()){
        
        if(IsKeyPressed(KEY_SPACE)) r.add_force_at_point({0.0, 1.0, 0.0}, r.position + Math::Vector3{1, 0, 0});

        r.integrate(GetFrameTime());
        UpdateCameraCustom(camera);

        BeginDrawing();
            ClearBackground(RAYWHITE);

            BeginMode3D(camera);
                
                DrawGrid(100, 1.0f);

                rlPushMatrix();
                    rlTranslatef((float)r.position.x, (float)r.position.y, (float)r.position.z);
                    rlMultMatrixf(MatrixToFloatV(QuaternionToRaylibMatrix(r.orientation)).v);

                    DrawCube({0, 0, 0}, 2.0f, 2.0f, 2.0f, RED);
                    DrawCubeWires({0, 0, 0}, 2.0f, 2.0f, 2.0f, BLACK);

                    DrawVector3D({0,0,0}, {1.5f, 0, 0}, RED);
                    DrawVector3D({0,0,0}, {0, 1.5f, 0}, GREEN);
                    DrawVector3D({0,0,0}, {0, 0, 1.5f}, BLUE);
                rlPopMatrix();
            EndMode3D();
        EndDrawing();


    }   

    CloseWindow();
    return 0;
}