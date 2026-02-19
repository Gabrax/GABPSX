#pragma once

#include <raylib.h>
#include <math.h>
#include <string>
#include <iomanip>
#include <sstream>


  static const float G = 1.0f;          
  static const float TIME_STEP = 5.0f; 
  enum IntegrationMethod { EULER, RK4 };
  static IntegrationMethod currentMethod = RK4;

  struct PointMass {
      Vector2 position;  
      Vector2 velocity;  
      float mass;        

      static inline void UpdateEuler(PointMass& body1, PointMass& body2, float dt) {
          Vector2 forceOn1 = ComputeGravitationalForce(body1, body2);
          Vector2 forceOn2 = { -forceOn1.x, -forceOn1.y };

          Vector2 acceleration1 = { forceOn1.x / body1.mass, forceOn1.y / body1.mass };
          Vector2 acceleration2 = { forceOn2.x / body2.mass, forceOn2.y / body2.mass };

          body1.velocity.x += acceleration1.x * dt;
          body1.velocity.y += acceleration1.y * dt;
          body2.velocity.x += acceleration2.x * dt;
          body2.velocity.y += acceleration2.y * dt;

          body1.position.x += body1.velocity.x * dt;
          body1.position.y += body1.velocity.y * dt;
          body2.position.x += body2.velocity.x * dt;
          body2.position.y += body2.velocity.y * dt;
      }

      static inline void UpdateRK4(PointMass& body1, PointMass& body2, float dt) {
        auto ComputeAcceleration = [](const PointMass& body, const PointMass& other) -> Vector2 {
            Vector2 force = PointMass::ComputeGravitationalForce(body, other);
            return { force.x / body.mass, force.y / body.mass };
        };

        Vector2 k1v1 = ComputeAcceleration(body1, body2);
        Vector2 k1p1 = body1.velocity;
        Vector2 k1v2 = ComputeAcceleration(body2, body1);
        Vector2 k1p2 = body2.velocity;

        Vector2 midVelocity1 = { body1.velocity.x + 0.5f * k1v1.x * dt, body1.velocity.y + 0.5f * k1v1.y * dt };
        Vector2 midPosition1 = { body1.position.x + 0.5f * k1p1.x * dt, body1.position.y + 0.5f * k1p1.y * dt };
        Vector2 midVelocity2 = { body2.velocity.x + 0.5f * k1v2.x * dt, body2.velocity.y + 0.5f * k1v2.y * dt };
        Vector2 midPosition2 = { body2.position.x + 0.5f * k1p2.x * dt, body2.position.y + 0.5f * k1p2.y * dt };

        Vector2 k2v1 = ComputeAcceleration({ midPosition1, midVelocity1, body1.mass }, { midPosition2, midVelocity2, body2.mass });
        Vector2 k2p1 = midVelocity1;
        Vector2 k2v2 = ComputeAcceleration({ midPosition2, midVelocity2, body2.mass }, { midPosition1, midVelocity1, body1.mass });
        Vector2 k2p2 = midVelocity2;

        Vector2 midVelocity1_2 = { body1.velocity.x + 0.5f * k2v1.x * dt, body1.velocity.y + 0.5f * k2v1.y * dt };
        Vector2 midPosition1_2 = { body1.position.x + 0.5f * k2p1.x * dt, body1.position.y + 0.5f * k2p1.y * dt };
        Vector2 midVelocity2_2 = { body2.velocity.x + 0.5f * k2v2.x * dt, body2.velocity.y + 0.5f * k2v2.y * dt };
        Vector2 midPosition2_2 = { body2.position.x + 0.5f * k2p2.x * dt, body2.position.y + 0.5f * k2p2.y * dt };

        Vector2 k3v1 = ComputeAcceleration({ midPosition1_2, midVelocity1_2, body1.mass }, { midPosition2_2, midVelocity2_2, body2.mass });
        Vector2 k3p1 = midVelocity1_2;
        Vector2 k3v2 = ComputeAcceleration({ midPosition2_2, midVelocity2_2, body2.mass }, { midPosition1_2, midVelocity1_2, body1.mass });
        Vector2 k3p2 = midVelocity2_2;

        Vector2 endVelocity1 = { body1.velocity.x + k3v1.x * dt, body1.velocity.y + k3v1.y * dt };
        Vector2 endPosition1 = { body1.position.x + k3p1.x * dt, body1.position.y + k3p1.y * dt };
        Vector2 endVelocity2 = { body2.velocity.x + k3v2.x * dt, body2.velocity.y + k3v2.y * dt };
        Vector2 endPosition2 = { body2.position.x + k3p2.x * dt, body2.position.y + k3p2.y * dt };

        Vector2 k4v1 = ComputeAcceleration({ endPosition1, endVelocity1, body1.mass }, { endPosition2, endVelocity2, body2.mass });
        Vector2 k4p1 = endVelocity1;
        Vector2 k4v2 = ComputeAcceleration({ endPosition2, endVelocity2, body2.mass }, { endPosition1, endVelocity1, body1.mass });
        Vector2 k4p2 = endVelocity2;

        body1.velocity.x += (dt / 6.0f) * (k1v1.x + 2 * k2v1.x + 2 * k3v1.x + k4v1.x);
        body1.velocity.y += (dt / 6.0f) * (k1v1.y + 2 * k2v1.y + 2 * k3v1.y + k4v1.y);
        body1.position.x += (dt / 6.0f) * (k1p1.x + 2 * k2p1.x + 2 * k3p1.x + k4p1.x);
        body1.position.y += (dt / 6.0f) * (k1p1.y + 2 * k2p1.y + 2 * k3p1.y + k4p1.y);

        body2.velocity.x += (dt / 6.0f) * (k1v2.x + 2 * k2v2.x + 2 * k3v2.x + k4v2.x);
        body2.velocity.y += (dt / 6.0f) * (k1v2.y + 2 * k2v2.y + 2 * k3v2.y + k4v2.y);
        body2.position.x += (dt / 6.0f) * (k1p2.x + 2 * k2p2.x + 2 * k3p2.x + k4p2.x);
        body2.position.y += (dt / 6.0f) * (k1p2.y + 2 * k2p2.y + 2 * k3p2.y + k4p2.y);
      }

      static inline  Vector2 ComputeGravitationalForce(const PointMass& a, const PointMass& b) {
          Vector2 direction = { b.position.x - a.position.x, b.position.y - a.position.y };
          float distance = sqrt(direction.x * direction.x + direction.y * direction.y);
          if (distance < 1.0f) distance = 1.0f; // Avoid division by zero
          float forceMagnitude = (G * a.mass * b.mass) / (distance * distance);
          return { direction.x / distance * forceMagnitude, direction.y / distance * forceMagnitude };
      }

      inline float ComputeKineticEnergy() const {
          float speedSquared = velocity.x * velocity.x + velocity.y * velocity.y;
          return 0.5f * mass * speedSquared;
      }
  };

  inline float ComputePotentialEnergy(const PointMass& a, const PointMass& b) {
      Vector2 direction = { b.position.x - a.position.x, b.position.y - a.position.y };
      float distance = sqrt(direction.x * direction.x + direction.y * direction.y);
      if (distance < 1.0f) distance = 1.0f; // Avoid division by zero
      return -(G * a.mass * b.mass) / distance;
  }

  inline std::string FormatFloat(float value, int precision = 3) {
      std::ostringstream stream;
      stream << std::fixed << std::setprecision(precision) << value;
      return stream.str();
  }

  struct CFF
  {
      CFF() 
      {
          centerMass = {400, 300};

          body1 = { { centerMass.x - 100, centerMass.y }, { 0, 0 }, 10.0f };
          body2 = { { centerMass.x + 100, centerMass.y }, { 0, 0 }, 10.0f };

          float distance1 = sqrt(pow(body2.position.x - body1.position.x, 2) + pow(body2.position.y - body1.position.y, 2));
          float orbitalSpeed1 = sqrt(G * (body1.mass + body2.mass) / distance1);

          body1.velocity = { 0, -orbitalSpeed1 * (body2.mass / (body1.mass + body2.mass)) };
          body2.velocity = { 0, orbitalSpeed1 * (body1.mass / (body1.mass + body2.mass)) };
      }

      inline void Render()
      {
          if (IsKeyPressed(KEY_ONE)) currentMethod = EULER;
          if (IsKeyPressed(KEY_TWO)) currentMethod = RK4;

          Vector2 forceOn1 = PointMass::ComputeGravitationalForce(body1, body2);
          Vector2 forceOn2 = { -forceOn1.x, -forceOn1.y };

          if (currentMethod == EULER) {
              PointMass::UpdateEuler(body1, body2, TIME_STEP);
          } else if (currentMethod == RK4) {
              PointMass::UpdateRK4(body1, body2, TIME_STEP);
          }

          float kineticEnergy = body1.ComputeKineticEnergy() + body2.ComputeKineticEnergy();
          float potentialEnergy = ComputePotentialEnergy(body1, body2);
          float totalEnergy = kineticEnergy + potentialEnergy;

          BeginDrawing();
          ClearBackground(RAYWHITE);

          DrawCircleV(body1.position, 10, RED);
          DrawCircleV(body2.position, 10, BLUE);

          DrawText(currentMethod == EULER ? "Method: Euler" : "Method: RK4", 10, 10, 20, BLACK);
          DrawText("Press 1 for Euler, 2 for RK4", 10, 40, 20, BLACK);
          DrawText(("Kinetic Energy: " + FormatFloat(kineticEnergy)).c_str(), 10, 80, 20, BLACK);
          DrawText(("Potential Energy: " + FormatFloat(potentialEnergy)).c_str(), 10, 110, 20, BLACK);
          DrawText(("Total Energy: " + FormatFloat(totalEnergy)).c_str(), 10, 140, 20, BLACK);

          DrawText("BACKSPACE to return, R to reset positions", 10, GetScreenHeight() - 25, 20, BLACK);
          EndDrawing();
      }

  private:
      Vector2 centerMass;
      PointMass body1;
      PointMass body2;
  };
