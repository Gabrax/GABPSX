#pragma once

#include "raylib.h"
#include <cmath>
#include <string>

namespace CollisionPrecision {

    static Vector2 pos1 = {200, 300};
    static Vector2 pos2 = {700, 281};
    static Vector2 velocity1 = {300, 0}; // High speed
    static Vector2 velocity2 = {0, 0}; // High speed
    static float radius = 20;
    static float mass1 = 1.0f;
    static float mass2 = 1.0f;

    inline float distance(Vector2 v1, Vector2 v2) {
        return (float) sqrt(pow(v2.x - v1.x, 2) + pow(v2.y - v1.y, 2));
    }

    inline Vector2 subtract(Vector2 v1, Vector2 v2) {
        return {v1.x - v2.x, v1.y - v2.y};
    }

    inline float dotProduct(Vector2 v1, Vector2 v2) {
        return v1.x * v2.x + v1.y * v2.y;
    }

    inline Vector2 normalize(Vector2 v) {
        float length = (float) sqrt(v.x * v.x + v.y * v.y);
        return {v.x / length, v.y / length};
    }

    inline bool isCollidingPrecise(Vector2 pos1, Vector2 pos2, Vector2 velocity1, Vector2 velocity2, float radius, float dt)
    {
        Vector2 relativeVelocity = subtract(velocity2, velocity1);
        Vector2 relativePosition = subtract(pos2, pos1);

        float a = dotProduct(relativeVelocity, relativeVelocity); // Quadratic term
        float b = 2 * dotProduct(relativePosition, relativeVelocity); // Linear term
        float c = dotProduct(relativePosition, relativePosition) - (4 * radius * radius); // Constant term

        float discriminant = b * b - 4 * a * c;

        if (discriminant < 0) {
            // No collision
            return false;
        }

        float sqrtDiscriminant = (float) sqrt(discriminant);
        float t1 = (-b - sqrtDiscriminant) / (2 * a);
        float t2 = (-b + sqrtDiscriminant) / (2 * a);

        // Check if the collision happens within the current frame (dt)
        return (t1 >= 0 && t1 <= dt) || (t2 >= 0 && t2 <= dt);
    }

    inline float findCollisionTime(Vector2 pos1, Vector2 pos2, Vector2 velocity1, Vector2 velocity2, float radius, float dt)
    {
        Vector2 relativeVelocity = subtract(velocity2, velocity1);
        Vector2 relativePosition = subtract(pos2, pos1);

        float a = dotProduct(relativeVelocity, relativeVelocity);
        float b = 2 * dotProduct(relativePosition, relativeVelocity);
        float c = dotProduct(relativePosition, relativePosition) - (4 * radius * radius);

        float discriminant = b * b - 4 * a * c;

        float sqrtDiscriminant = (float) sqrt(discriminant);
        float t1 = (-b - sqrtDiscriminant) / (2 * a);
        float t2 = (-b + sqrtDiscriminant) / (2 * a);

        return (t1 >= 0 && t1 <= dt) ? t1 : t2;
    }

    inline void resolveCollision(Vector2& pos1, Vector2& pos2, Vector2& v1, Vector2& v2, float m1, float m2)
    {
        Vector2 collisionNormal = normalize(subtract(pos2, pos1));

        float v1Normal = dotProduct(v1, collisionNormal);
        float v2Normal = dotProduct(v2, collisionNormal);

        float v1New = ((v1Normal * (m1 - m2)) + (2 * m2 * v2Normal)) / (m1 + m2);
        float v2New = ((v2Normal * (m2 - m1)) + (2 * m1 * v1Normal)) / (m1 + m2);

        v1.x = collisionNormal.x * v1New + v1.x - collisionNormal.x * v1Normal;
        v1.y = collisionNormal.y * v1New + v1.y - collisionNormal.y * v1Normal;
        v2.x = collisionNormal.x * v2New + v2.x - collisionNormal.x * v2Normal;
        v2.y = collisionNormal.y * v2New + v2.y - collisionNormal.y * v2Normal;

        float overlap = (2 * radius) - distance(pos1, pos2);
        pos1.x = pos1.x - collisionNormal.x * overlap * 0.5f;
        pos1.y = pos1.y - collisionNormal.y * overlap * 0.5f;
        pos2.x = pos2.x + collisionNormal.x * overlap * 0.5f;
        pos2.y = pos2.y + collisionNormal.y * overlap * 0.5f;
    }

    inline void Render()
    {
        float dt = GetFrameTime();

        if (isCollidingPrecise(pos1, pos2, velocity1, velocity2, radius, dt)) {

            float collisionTime = findCollisionTime(pos1, pos2, velocity1, velocity2, radius, dt);

            pos1.x = pos1.x + velocity1.x * collisionTime;
            pos1.y = pos1.y + velocity1.y * collisionTime;
            pos2.x = pos2.x + velocity2.x * collisionTime;
            pos2.y = pos2.y + velocity2.y * collisionTime;

            resolveCollision(pos1, pos2, velocity1, velocity2, mass1, mass2);

            float remainingTime = dt - collisionTime;
            pos1.x = pos1.x + velocity1.x * remainingTime;
            pos1.y = pos1.y + velocity1.y * remainingTime;
            pos2.x = pos2.x + velocity2.x * remainingTime;
            pos2.y = pos2.y + velocity2.y * remainingTime;
        } else {

            pos1.x = pos1.x + velocity1.x * dt;
            pos1.y = pos1.y + velocity1.y * dt;
            pos2.x = pos2.x + velocity2.x * dt;
            pos2.y = pos2.y + velocity2.y * dt;
        }

        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawCircleV(pos1, radius, BLUE);
        DrawCircleV(pos2, radius, RED);

        std::string text1 = "Circ1velX: " + std::to_string(velocity1.x) + ", Circ1velY: " + std::to_string(velocity1.y);
        std::string text2 = "Circ2velX: " + std::to_string(velocity2.x) + ", Circ2velY: " + std::to_string(velocity2.y);

        DrawText(text1.c_str(), 10, 10, 20, BLACK);
        DrawText(text2.c_str(), 10, 40, 20, BLACK);
  
        DrawText("BACKSPACE to return, R to reset positions", 10, GetScreenHeight() - 25, 20, BLACK);
        EndDrawing();

        if (IsKeyPressed(KEY_R)) {
            pos1 = {200, 300};
            pos2 = {700, 261};
            velocity1 = {300, 0};
            velocity2 = {0, 0};
        }
      }
}
