#include <raylib.h>
#include <math.h>
#include <stdio.h>

typedef enum {
    RK1,
    RK2, RK2_Heun, RK2_Ralston,
    RK3, RK3_Heun, RK3_Ralston, RK3_HouwenWray, RK3_Strong_Stability_Preserving,
    RK4, RK4_3_8, RK4_Ralston,

    METHOD_COUNT
} Method;
static Method currentMethod = RK1;

const char* methodNames[] = {
    "RK1",
    "RK2 Midpoint",
    "RK2 Heun",
    "RK2 Ralston",
    "Rk3",
    "RK3 Heun",
    "RK3 Ralston",
    "RK3 Houwen-Wray",
    "RK3 SSP",
    "RK4",
    "RK4 3/8",
    "RK4 Ralston"
};

static const float G = 1.0f;
static const float TIME_STEP = 60.0f;

typedef struct {
    Vector2 position;
    Vector2 velocity;
    float mass;
} Object;

Vector2 ComputeGravitationalForce(const Object* a, const Object* b)
{
    const Vector2 direction = (Vector2){ b->position.x - a->position.x, b->position.y - a->position.y };
    float distance = sqrtf(direction.x * direction.x + direction.y * direction.y);
    if (distance < 1.0f) distance = 1.0f; // Avoid division by zero
    const float forceMagnitude = (G * a->mass * b->mass) / (distance * distance);
    return (Vector2){ direction.x / distance * forceMagnitude, direction.y / distance * forceMagnitude };
}

Vector2 ComputeAcceleration(const Object* body1, const Object* body2)
{
    const Vector2 force = ComputeGravitationalForce(body1, body2);
    return (Vector2){ force.x / body1->mass, force.y / body1->mass };
}

void UpdateRK1(Object* body1, Object* body2, const float dt)
{
    const Vector2 force = ComputeGravitationalForce(body1, body2);

    const Vector2 acceleration1 = (Vector2){ force.x / body1->mass, force.y / body1->mass };
    const Vector2 acceleration2 = (Vector2){ -force.x / body2->mass, -force.y / body2->mass };

    body1->velocity.x += acceleration1.x * dt;
    body1->velocity.y += acceleration1.y * dt;
    body2->velocity.x += acceleration2.x * dt;
    body2->velocity.y += acceleration2.y * dt;

    body1->position.x += body1->velocity.x * dt;
    body1->position.y += body1->velocity.y * dt;
    body2->position.x += body2->velocity.x * dt;
    body2->position.y += body2->velocity.y * dt;
}

void UpdateRK2_Midpoint(Object* body1, Object* body2, const float dt)
{
    const Vector2 k1v1 = ComputeAcceleration(body1, body2);
    const Vector2 k1p1 = body1->velocity;

    const Vector2 k1v2 = ComputeAcceleration(body2, body1);
    const Vector2 k1p2 = body2->velocity;

    const Object mid1 = {
        { body1->position.x + 0.5f * k1p1.x * dt, body1->position.y + 0.5f * k1p1.y * dt },
        { body1->velocity.x + 0.5f * k1v1.x * dt, body1->velocity.y + 0.5f * k1v1.y * dt },
        body1->mass
    };

    const Object mid2 = {
        { body2->position.x + 0.5f * k1p2.x * dt, body2->position.y + 0.5f * k1p2.y * dt },
        { body2->velocity.x + 0.5f * k1v2.x * dt, body2->velocity.y + 0.5f * k1v2.y * dt },
        body2->mass
    };

    const Vector2 k2v1 = ComputeAcceleration(&mid1, &mid2);
    const Vector2 k2p1 = mid1.velocity;

    const Vector2 k2v2 = ComputeAcceleration(&mid2, &mid1);
    const Vector2 k2p2 = mid2.velocity;

    body1->velocity.x += dt * k2v1.x;
    body1->velocity.y += dt * k2v1.y;
    body1->position.x += dt * k2p1.x;
    body1->position.y += dt * k2p1.y;

    body2->velocity.x += dt * k2v2.x;
    body2->velocity.y += dt * k2v2.y;
    body2->position.x += dt * k2p2.x;
    body2->position.y += dt * k2p2.y;
}

void UpdateRK2_Heun(Object* body1, Object* body2, const float dt)
{
    const Vector2 k1v1 = ComputeAcceleration(body1, body2);
    const Vector2 k1p1 = body1->velocity;

    const Vector2 k1v2 = ComputeAcceleration(body2, body1);
    const Vector2 k1p2 = body2->velocity;

    const Object end1 = {
        { body1->position.x + k1p1.x * dt, body1->position.y + k1p1.y * dt },
        { body1->velocity.x + k1v1.x * dt, body1->velocity.y + k1v1.y * dt },
        body1->mass
    };

    const Object end2 = {
        { body2->position.x + k1p2.x * dt, body2->position.y + k1p2.y * dt },
        { body2->velocity.x + k1v2.x * dt, body2->velocity.y + k1v2.y * dt },
        body2->mass
    };

    const Vector2 k2v1 = ComputeAcceleration(&end1, &end2);
    const Vector2 k2p1 = end1.velocity;

    const Vector2 k2v2 = ComputeAcceleration(&end2, &end1);
    const Vector2 k2p2 = end2.velocity;

    body1->velocity.x += (dt * 0.5f) * (k1v1.x + k2v1.x);
    body1->velocity.y += (dt * 0.5f) * (k1v1.y + k2v1.y);
    body1->position.x += (dt * 0.5f) * (k1p1.x + k2p1.x);
    body1->position.y += (dt * 0.5f) * (k1p1.y + k2p1.y);

    body2->velocity.x += (dt * 0.5f) * (k1v2.x + k2v2.x);
    body2->velocity.y += (dt * 0.5f) * (k1v2.y + k2v2.y);
    body2->position.x += (dt * 0.5f) * (k1p2.x + k2p2.x);
    body2->position.y += (dt * 0.5f) * (k1p2.y + k2p2.y);
}

void UpdateRK2_Ralston(Object* body1, Object* body2, const float dt)
{
    const Vector2 k1v1 = ComputeAcceleration(body1, body2);
    const Vector2 k1p1 = body1->velocity;

    const Vector2 k1v2 = ComputeAcceleration(body2, body1);
    const Vector2 k1p2 = body2->velocity;

    const Object step1 = {
        { body1->position.x + (2.0f / 3.0f) * k1p1.x * dt,
          body1->position.y + (2.0f / 3.0f) * k1p1.y * dt },
        { body1->velocity.x + (2.0f / 3.0f) * k1v1.x * dt,
          body1->velocity.y + (2.0f / 3.0f) * k1v1.y * dt },
        body1->mass
    };

    const Object step2 = {
        { body2->position.x + (2.0f / 3.0f) * k1p2.x * dt,
          body2->position.y + (2.0f / 3.0f) * k1p2.y * dt },
        { body2->velocity.x + (2.0f / 3.0f) * k1v2.x * dt,
          body2->velocity.y + (2.0f / 3.0f) * k1v2.y * dt },
        body2->mass
    };

    const Vector2 k2v1 = ComputeAcceleration(&step1, &step2);
    const Vector2 k2p1 = step1.velocity;

    const Vector2 k2v2 = ComputeAcceleration(&step2, &step1);
    const Vector2 k2p2 = step2.velocity;

    body1->velocity.x += dt * (0.25f * k1v1.x + 0.75f * k2v1.x);
    body1->velocity.y += dt * (0.25f * k1v1.y + 0.75f * k2v1.y);
    body1->position.x += dt * (0.25f * k1p1.x + 0.75f * k2p1.x);
    body1->position.y += dt * (0.25f * k1p1.y + 0.75f * k2p1.y);

    body2->velocity.x += dt * (0.25f * k1v2.x + 0.75f * k2v2.x);
    body2->velocity.y += dt * (0.25f * k1v2.y + 0.75f * k2v2.y);
    body2->position.x += dt * (0.25f * k1p2.x + 0.75f * k2p2.x);
    body2->position.y += dt * (0.25f * k1p2.y + 0.75f * k2p2.y);
}

void UpdateRK3_Classic(Object* body1, Object* body2, const float dt)
{
    const Vector2 k1v1 = ComputeAcceleration(body1, body2);
    const Vector2 k1p1 = body1->velocity;

    const Vector2 k1v2 = ComputeAcceleration(body2, body1);
    const Vector2 k1p2 = body2->velocity;

    const Object mid1 = {
        { body1->position.x + 0.5f * k1p1.x * dt, body1->position.y + 0.5f * k1p1.y * dt },
        { body1->velocity.x + 0.5f * k1v1.x * dt, body1->velocity.y + 0.5f * k1v1.y * dt },
        body1->mass
    };

    const Object mid2 = {
        { body2->position.x + 0.5f * k1p2.x * dt, body2->position.y + 0.5f * k1p2.y * dt },
        { body2->velocity.x + 0.5f * k1v2.x * dt, body2->velocity.y + 0.5f * k1v2.y * dt },
        body2->mass
    };

    const Vector2 k2v1 = ComputeAcceleration(&mid1, &mid2);
    const Vector2 k2p1 = mid1.velocity;

    const Vector2 k2v2 = ComputeAcceleration(&mid2, &mid1);
    const Vector2 k2p2 = mid2.velocity;

    const Object end1 = {
        { body1->position.x + dt * (-k1p1.x + 2.0f * k2p1.x),
          body1->position.y + dt * (-k1p1.y + 2.0f * k2p1.y) },
        { body1->velocity.x + dt * (-k1v1.x + 2.0f * k2v1.x),
          body1->velocity.y + dt * (-k1v1.y + 2.0f * k2v1.y) },
        body1->mass
    };

    const Object end2 = {
        { body2->position.x + dt * (-k1p2.x + 2.0f * k2p2.x),
          body2->position.y + dt * (-k1p2.y + 2.0f * k2p2.y) },
        { body2->velocity.x + dt * (-k1v2.x + 2.0f * k2v2.x),
          body2->velocity.y + dt * (-k1v2.y + 2.0f * k2v2.y) },
        body2->mass
    };

    const Vector2 k3v1 = ComputeAcceleration(&end1, &end2);
    const Vector2 k3p1 = end1.velocity;

    const Vector2 k3v2 = ComputeAcceleration(&end2, &end1);
    const Vector2 k3p2 = end2.velocity;

    body1->velocity.x += (dt / 6.0f) * (k1v1.x + 4.0f * k2v1.x + k3v1.x);
    body1->velocity.y += (dt / 6.0f) * (k1v1.y + 4.0f * k2v1.y + k3v1.y);
    body1->position.x += (dt / 6.0f) * (k1p1.x + 4.0f * k2p1.x + k3p1.x);
    body1->position.y += (dt / 6.0f) * (k1p1.y + 4.0f * k2p1.y + k3p1.y);

    body2->velocity.x += (dt / 6.0f) * (k1v2.x + 4.0f * k2v2.x + k3v2.x);
    body2->velocity.y += (dt / 6.0f) * (k1v2.y + 4.0f * k2v2.y + k3v2.y);
    body2->position.x += (dt / 6.0f) * (k1p2.x + 4.0f * k2p2.x + k3p2.x);
    body2->position.y += (dt / 6.0f) * (k1p2.y + 4.0f * k2p2.y + k3p2.y);
}

void UpdateRK3_Heun(Object* body1, Object* body2, const float dt)
{
    Vector2 k1v1 = ComputeAcceleration(body1, body2);
    Vector2 k1p1 = body1->velocity;

    Vector2 k1v2 = ComputeAcceleration(body2, body1);
    Vector2 k1p2 = body2->velocity;

    Object mid1 = {
        { body1->position.x + (dt / 3.0f) * k1p1.x,
          body1->position.y + (dt / 3.0f) * k1p1.y },
        { body1->velocity.x + (dt / 3.0f) * k1v1.x,
          body1->velocity.y + (dt / 3.0f) * k1v1.y },
        body1->mass
    };

    Object mid2 = {
        { body2->position.x + (dt / 3.0f) * k1p2.x,
          body2->position.y + (dt / 3.0f) * k1p2.y },
        { body2->velocity.x + (dt / 3.0f) * k1v2.x,
          body2->velocity.y + (dt / 3.0f) * k1v2.y },
        body2->mass
    };

    Vector2 k2v1 = ComputeAcceleration(&mid1, &mid2);
    Vector2 k2p1 = mid1.velocity;

    Vector2 k2v2 = ComputeAcceleration(&mid2, &mid1);
    Vector2 k2p2 = mid2.velocity;

    Object end1 = {
        { body1->position.x + (2.0f * dt / 3.0f) * k2p1.x,
          body1->position.y + (2.0f * dt / 3.0f) * k2p1.y },
        { body1->velocity.x + (2.0f * dt / 3.0f) * k2v1.x,
          body1->velocity.y + (2.0f * dt / 3.0f) * k2v1.y },
        body1->mass
    };

    Object end2 = {
        { body2->position.x + (2.0f * dt / 3.0f) * k2p2.x,
          body2->position.y + (2.0f * dt / 3.0f) * k2p2.y },
        { body2->velocity.x + (2.0f * dt / 3.0f) * k2v2.x,
          body2->velocity.y + (2.0f * dt / 3.0f) * k2v2.y },
        body2->mass
    };

    Vector2 k3v1 = ComputeAcceleration(&end1, &end2);
    Vector2 k3p1 = end1.velocity;

    Vector2 k3v2 = ComputeAcceleration(&end2, &end1);
    Vector2 k3p2 = end2.velocity;

    body1->velocity.x += dt * (0.25f * k1v1.x + 0.75f * k3v1.x);
    body1->velocity.y += dt * (0.25f * k1v1.y + 0.75f * k3v1.y);
    body1->position.x += dt * (0.25f * k1p1.x + 0.75f * k3p1.x);
    body1->position.y += dt * (0.25f * k1p1.y + 0.75f * k3p1.y);

    body2->velocity.x += dt * (0.25f * k1v2.x + 0.75f * k3v2.x);
    body2->velocity.y += dt * (0.25f * k1v2.y + 0.75f * k3v2.y);
    body2->position.x += dt * (0.25f * k1p2.x + 0.75f * k3p2.x);
    body2->position.y += dt * (0.25f * k1p2.y + 0.75f * k3p2.y);
}

void UpdateRK3_Ralston(Object* body1, Object* body2, const float dt)
{
    Vector2 k1v1 = ComputeAcceleration(body1, body2);
    Vector2 k1p1 = body1->velocity;

    Vector2 k1v2 = ComputeAcceleration(body2, body1);
    Vector2 k1p2 = body2->velocity;

    Object s1 = {
        { body1->position.x + 0.5f * dt * k1p1.x,
          body1->position.y + 0.5f * dt * k1p1.y },
        { body1->velocity.x + 0.5f * dt * k1v1.x,
          body1->velocity.y + 0.5f * dt * k1v1.y },
        body1->mass
    };

    Object s2 = {
        { body2->position.x + 0.5f * dt * k1p2.x,
          body2->position.y + 0.5f * dt * k1p2.y },
        { body2->velocity.x + 0.5f * dt * k1v2.x,
          body2->velocity.y + 0.5f * dt * k1v2.y },
        body2->mass
    };

    Vector2 k2v1 = ComputeAcceleration(&s1, &s2);
    Vector2 k2p1 = s1.velocity;

    Vector2 k2v2 = ComputeAcceleration(&s2, &s1);
    Vector2 k2p2 = s2.velocity;

    Object e1 = {
        { body1->position.x + dt * (-k1p1.x + 2.0f * k2p1.x),
          body1->position.y + dt * (-k1p1.y + 2.0f * k2p1.y) },
        { body1->velocity.x + dt * (-k1v1.x + 2.0f * k2v1.x),
          body1->velocity.y + dt * (-k1v1.y + 2.0f * k2v1.y) },
        body1->mass
    };

    Object e2 = {
        { body2->position.x + dt * (-k1p2.x + 2.0f * k2p2.x),
          body2->position.y + dt * (-k1p2.y + 2.0f * k2p2.y) },
        { body2->velocity.x + dt * (-k1v2.x + 2.0f * k2v2.x),
          body2->velocity.y + dt * (-k1v2.y + 2.0f * k2v2.y) },
        body2->mass
    };

    Vector2 k3v1 = ComputeAcceleration(&e1, &e2);
    Vector2 k3p1 = e1.velocity;

    Vector2 k3v2 = ComputeAcceleration(&e2, &e1);
    Vector2 k3p2 = e2.velocity;

    body1->velocity.x += (dt / 6.0f) * (k1v1.x + 4.0f * k2v1.x + k3v1.x);
    body1->velocity.y += (dt / 6.0f) * (k1v1.y + 4.0f * k2v1.y + k3v1.y);
    body1->position.x += (dt / 6.0f) * (k1p1.x + 4.0f * k2p1.x + k3p1.x);
    body1->position.y += (dt / 6.0f) * (k1p1.y + 4.0f * k2p1.y + k3p1.y);

    body2->velocity.x += (dt / 6.0f) * (k1v2.x + 4.0f * k2v2.x + k3v2.x);
    body2->velocity.y += (dt / 6.0f) * (k1v2.y + 4.0f * k2v2.y + k3v2.y);
    body2->position.x += (dt / 6.0f) * (k1p2.x + 4.0f * k2p2.x + k3p2.x);
    body2->position.y += (dt / 6.0f) * (k1p2.y + 4.0f * k2p2.y + k3p2.y);
}

void UpdateRK3_HouwenWray(Object* body1, Object* body2, const float dt)
{
    Vector2 k1v1 = ComputeAcceleration(body1, body2);
    Vector2 k1p1 = body1->velocity;

    Vector2 k1v2 = ComputeAcceleration(body2, body1);
    Vector2 k1p2 = body2->velocity;

    Object s1 = {
        { body1->position.x + dt * k1p1.x,
          body1->position.y + dt * k1p1.y },
        { body1->velocity.x + dt * k1v1.x,
          body1->velocity.y + dt * k1v1.y },
        body1->mass
    };

    Object s2 = {
        { body2->position.x + dt * k1p2.x,
          body2->position.y + dt * k1p2.y },
        { body2->velocity.x + dt * k1v2.x,
          body2->velocity.y + dt * k1v2.y },
        body2->mass
    };

    Vector2 k2v1 = ComputeAcceleration(&s1, &s2);
    Vector2 k2p1 = s1.velocity;

    Vector2 k2v2 = ComputeAcceleration(&s2, &s1);
    Vector2 k2p2 = s2.velocity;

    body1->velocity.x += dt * (0.5f * k1v1.x + 0.5f * k2v1.x);
    body1->velocity.y += dt * (0.5f * k1v1.y + 0.5f * k2v1.y);
    body1->position.x += dt * (0.5f * k1p1.x + 0.5f * k2p1.x);
    body1->position.y += dt * (0.5f * k1p1.y + 0.5f * k2p1.y);

    body2->velocity.x += dt * (0.5f * k1v2.x + 0.5f * k2v2.x);
    body2->velocity.y += dt * (0.5f * k1v2.y + 0.5f * k2v2.y);
    body2->position.x += dt * (0.5f * k1p2.x + 0.5f * k2p2.x);
    body2->position.y += dt * (0.5f * k1p2.y + 0.5f * k2p2.y);
}

void UpdateRK3_Strong_Stability_Preserving(Object* body1, Object* body2, const float dt)
{
    Object u1 = *body1;
    Object v1 = *body2;

    Vector2 a1 = ComputeAcceleration(&u1, &v1);
    Vector2 b1 = ComputeAcceleration(&v1, &u1);

    Object u2 = {
        { u1.position.x + dt * u1.velocity.x,
          u1.position.y + dt * u1.velocity.y },
        { u1.velocity.x + dt * a1.x,
          u1.velocity.y + dt * a1.y },
        u1.mass
    };

    Object v2 = {
        { v1.position.x + dt * v1.velocity.x,
          v1.position.y + dt * v1.velocity.y },
        { v1.velocity.x + dt * b1.x,
          v1.velocity.y + dt * b1.y },
        v1.mass
    };

    Vector2 a2 = ComputeAcceleration(&u2, &v2);
    Vector2 b2 = ComputeAcceleration(&v2, &u2);

    Object u3 = {
        { 0.75f * u1.position.x + 0.25f * (u2.position.x + dt * u2.velocity.x),
          0.75f * u1.position.y + 0.25f * (u2.position.y + dt * u2.velocity.y) },
        { 0.75f * u1.velocity.x + 0.25f * (u2.velocity.x + dt * a2.x),
          0.75f * u1.velocity.y + 0.25f * (u2.velocity.y + dt * a2.y) },
        u1.mass
    };

    Object v3 = {
        { 0.75f * v1.position.x + 0.25f * (v2.position.x + dt * v2.velocity.x),
          0.75f * v1.position.y + 0.25f * (v2.position.y + dt * v2.velocity.y) },
        { 0.75f * v1.velocity.x + 0.25f * (v2.velocity.x + dt * b2.x),
          0.75f * v1.velocity.y + 0.25f * (v2.velocity.y + dt * b2.y) },
        v1.mass
    };

    Vector2 a3 = ComputeAcceleration(&u3, &v3);
    Vector2 b3 = ComputeAcceleration(&v3, &u3);

    body1->position.x = (1.0f / 3.0f) * u1.position.x + (2.0f / 3.0f) * (u3.position.x + dt * u3.velocity.x);
    body1->position.y = (1.0f / 3.0f) * u1.position.y + (2.0f / 3.0f) * (u3.position.y + dt * u3.velocity.y);
    body1->velocity.x = (1.0f / 3.0f) * u1.velocity.x + (2.0f / 3.0f) * (u3.velocity.x + dt * a3.x);
    body1->velocity.y = (1.0f / 3.0f) * u1.velocity.y + (2.0f / 3.0f) * (u3.velocity.y + dt * a3.y);

    body2->position.x = (1.0f / 3.0f) * v1.position.x + (2.0f / 3.0f) * (v3.position.x + dt * v3.velocity.x);
    body2->position.y = (1.0f / 3.0f) * v1.position.y + (2.0f / 3.0f) * (v3.position.y + dt * v3.velocity.y);
    body2->velocity.x = (1.0f / 3.0f) * v1.velocity.x + (2.0f / 3.0f) * (v3.velocity.x + dt * b3.x);
    body2->velocity.y = (1.0f / 3.0f) * v1.velocity.y + (2.0f / 3.0f) * (v3.velocity.y + dt * b3.y);
}

void UpdateRK4(Object* body1, Object* body2, const float dt)
{
    Vector2 k1v1 = ComputeAcceleration(body1, body2);
    Vector2 k1p1 = body1->velocity;
    Vector2 k1v2 = ComputeAcceleration(body2, body1);
    Vector2 k1p2 = body2->velocity;

    Vector2 midVelocity1 = { body1->velocity.x + 0.5f * k1v1.x * dt, body1->velocity.y + 0.5f * k1v1.y * dt };
    Vector2 midPosition1 = { body1->position.x + 0.5f * k1p1.x * dt, body1->position.y + 0.5f * k1p1.y * dt };
    Vector2 midVelocity2 = { body2->velocity.x + 0.5f * k1v2.x * dt, body2->velocity.y + 0.5f * k1v2.y * dt };
    Vector2 midPosition2 = { body2->position.x + 0.5f * k1p2.x * dt, body2->position.y + 0.5f * k1p2.y * dt };

    Object mass1 = (Object){ midPosition1, midVelocity1, body1->mass };
    Object mass2 = (Object){ midPosition2, midVelocity2, body2->mass };

    Vector2 k2v1 = ComputeAcceleration(&mass1, &mass2);
    Vector2 k2p1 = midVelocity1;
    Vector2 k2v2 = ComputeAcceleration(&mass2, &mass1);
    Vector2 k2p2 = midVelocity2;

    Vector2 midVelocity1_2 = { body1->velocity.x + 0.5f * k2v1.x * dt, body1->velocity.y + 0.5f * k2v1.y * dt };
    Vector2 midPosition1_2 = { body1->position.x + 0.5f * k2p1.x * dt, body1->position.y + 0.5f * k2p1.y * dt };
    Vector2 midVelocity2_2 = { body2->velocity.x + 0.5f * k2v2.x * dt, body2->velocity.y + 0.5f * k2v2.y * dt };
    Vector2 midPosition2_2 = { body2->position.x + 0.5f * k2p2.x * dt, body2->position.y + 0.5f * k2p2.y * dt };

    mass1 = (Object){ midPosition1_2, midVelocity1_2, body1->mass };
    mass2 = (Object){ midPosition2_2, midVelocity2_2, body2->mass };

    Vector2 k3v1 = ComputeAcceleration(&mass1, &mass2);
    Vector2 k3p1 = midVelocity1_2;
    Vector2 k3v2 = ComputeAcceleration(&mass2, &mass1);
    Vector2 k3p2 = midVelocity2_2;

    Vector2 endVelocity1 = { body1->velocity.x + k3v1.x * dt, body1->velocity.y + k3v1.y * dt };
    Vector2 endPosition1 = { body1->position.x + k3p1.x * dt, body1->position.y + k3p1.y * dt };
    Vector2 endVelocity2 = { body2->velocity.x + k3v2.x * dt, body2->velocity.y + k3v2.y * dt };
    Vector2 endPosition2 = { body2->position.x + k3p2.x * dt, body2->position.y + k3p2.y * dt };

    mass1 = (Object){ endPosition1, endVelocity1, body1->mass };
    mass2 = (Object){ endPosition2, endVelocity2, body2->mass };

    Vector2 k4v1 = ComputeAcceleration(&mass1, &mass2);
    Vector2 k4p1 = endVelocity1;
    Vector2 k4v2 = ComputeAcceleration(&mass2, &mass1);
    Vector2 k4p2 = endVelocity2;

    body1->velocity.x += (dt / 6.0f) * (k1v1.x + 2 * k2v1.x + 2 * k3v1.x + k4v1.x);
    body1->velocity.y += (dt / 6.0f) * (k1v1.y + 2 * k2v1.y + 2 * k3v1.y + k4v1.y);
    body1->position.x += (dt / 6.0f) * (k1p1.x + 2 * k2p1.x + 2 * k3p1.x + k4p1.x);
    body1->position.y += (dt / 6.0f) * (k1p1.y + 2 * k2p1.y + 2 * k3p1.y + k4p1.y);

    body2->velocity.x += (dt / 6.0f) * (k1v2.x + 2 * k2v2.x + 2 * k3v2.x + k4v2.x);
    body2->velocity.y += (dt / 6.0f) * (k1v2.y + 2 * k2v2.y + 2 * k3v2.y + k4v2.y);
    body2->position.x += (dt / 6.0f) * (k1p2.x + 2 * k2p2.x + 2 * k3p2.x + k4p2.x);
    body2->position.y += (dt / 6.0f) * (k1p2.y + 2 * k2p2.y + 2 * k3p2.y + k4p2.y);
}

void UpdateRK4_3_8(Object* body1, Object* body2, const float dt)
{
    Vector2 k1v1 = ComputeAcceleration(body1, body2);
    Vector2 k1p1 = body1->velocity;

    Vector2 k1v2 = ComputeAcceleration(body2, body1);
    Vector2 k1p2 = body2->velocity;

    Object s1_1 = {
        { body1->position.x + (dt / 3.0f) * k1p1.x,
          body1->position.y + (dt / 3.0f) * k1p1.y },
        { body1->velocity.x + (dt / 3.0f) * k1v1.x,
          body1->velocity.y + (dt / 3.0f) * k1v1.y },
        body1->mass
    };

    Object s2_1 = {
        { body2->position.x + (dt / 3.0f) * k1p2.x,
          body2->position.y + (dt / 3.0f) * k1p2.y },
        { body2->velocity.x + (dt / 3.0f) * k1v2.x,
          body2->velocity.y + (dt / 3.0f) * k1v2.y },
        body2->mass
    };

    Vector2 k2v1 = ComputeAcceleration(&s1_1, &s2_1);
    Vector2 k2p1 = s1_1.velocity;

    Vector2 k2v2 = ComputeAcceleration(&s2_1, &s1_1);
    Vector2 k2p2 = s2_1.velocity;

    Object s1_2 = {
        { body1->position.x + dt * (-k1p1.x/3.0f + k2p1.x),
          body1->position.y + dt * (-k1p1.y/3.0f + k2p1.y) },
        { body1->velocity.x + dt * (-k1v1.x/3.0f + k2v1.x),
          body1->velocity.y + dt * (-k1v1.y/3.0f + k2v1.y) },
        body1->mass
    };

    Object s2_2 = {
        { body2->position.x + dt * (-k1p2.x/3.0f + k2p2.x),
          body2->position.y + dt * (-k1p2.y/3.0f + k2p2.y) },
        { body2->velocity.x + dt * (-k1v2.x/3.0f + k2v2.x),
          body2->velocity.y + dt * (-k1v2.y/3.0f + k2v2.y) },
        body2->mass
    };

    Vector2 k3v1 = ComputeAcceleration(&s1_2, &s2_2);
    Vector2 k3p1 = s1_2.velocity;

    Vector2 k3v2 = ComputeAcceleration(&s2_2, &s1_2);
    Vector2 k3p2 = s2_2.velocity;

    Object s1_3 = {
        { body1->position.x + dt * (k1p1.x - k2p1.x + k3p1.x),
          body1->position.y + dt * (k1p1.y - k2p1.y + k3p1.y) },
        { body1->velocity.x + dt * (k1v1.x - k2v1.x + k3v1.x),
          body1->velocity.y + dt * (k1v1.y - k2v1.y + k3v1.y) },
        body1->mass
    };

    Object s2_3 = {
        { body2->position.x + dt * (k1p2.x - k2p2.x + k3p2.x),
          body2->position.y + dt * (k1p2.y - k2p2.y + k3p2.y) },
        { body2->velocity.x + dt * (k1v2.x - k2v2.x + k3v2.x),
          body2->velocity.y + dt * (k1v2.y - k2v2.y + k3v2.y) },
        body2->mass
    };

    Vector2 k4v1 = ComputeAcceleration(&s1_3, &s2_3);
    Vector2 k4p1 = s1_3.velocity;

    Vector2 k4v2 = ComputeAcceleration(&s2_3, &s1_3);
    Vector2 k4p2 = s2_3.velocity;

    body1->velocity.x += (dt / 8.0f) * (k1v1.x + 3*k2v1.x + 3*k3v1.x + k4v1.x);
    body1->velocity.y += (dt / 8.0f) * (k1v1.y + 3*k2v1.y + 3*k3v1.y + k4v1.y);
    body1->position.x += (dt / 8.0f) * (k1p1.x + 3*k2p1.x + 3*k3p1.x + k4p1.x);
    body1->position.y += (dt / 8.0f) * (k1p1.y + 3*k2p1.y + 3*k3p1.y + k4p1.y);

    body2->velocity.x += (dt / 8.0f) * (k1v2.x + 3*k2v2.x + 3*k3v2.x + k4v2.x);
    body2->velocity.y += (dt / 8.0f) * (k1v2.y + 3*k2v2.y + 3*k3v2.y + k4v2.y);
    body2->position.x += (dt / 8.0f) * (k1p2.x + 3*k2p2.x + 3*k3p2.x + k4p2.x);
    body2->position.y += (dt / 8.0f) * (k1p2.y + 3*k2p2.y + 3*k3p2.y + k4p2.y);
}

void UpdateRK4_Ralston(Object* body1, Object* body2, const float dt)
{
    Vector2 k1v1 = ComputeAcceleration(body1, body2);
    Vector2 k1p1 = body1->velocity;

    Vector2 k1v2 = ComputeAcceleration(body2, body1);
    Vector2 k1p2 = body2->velocity;

    Object s1_2 = {
        { body1->position.x + 0.4f * dt * k1p1.x,
          body1->position.y + 0.4f * dt * k1p1.y },
        { body1->velocity.x + 0.4f * dt * k1v1.x,
          body1->velocity.y + 0.4f * dt * k1v1.y },
        body1->mass
    };

    Object s2_2 = {
        { body2->position.x + 0.4f * dt * k1p2.x,
          body2->position.y + 0.4f * dt * k1p2.y },
        { body2->velocity.x + 0.4f * dt * k1v2.x,
          body2->velocity.y + 0.4f * dt * k1v2.y },
        body2->mass
    };

    Vector2 k2v1 = ComputeAcceleration(&s1_2, &s2_2);
    Vector2 k2p1 = s1_2.velocity;

    Vector2 k2v2 = ComputeAcceleration(&s2_2, &s1_2);
    Vector2 k2p2 = s2_2.velocity;

    Object s1_3 = {
        { body1->position.x + dt * (0.29697761f * k1p1.x + 0.15875964f * k2p1.x),
          body1->position.y + dt * (0.29697761f * k1p1.y + 0.15875964f * k2p1.y) },
        { body1->velocity.x + dt * (0.29697761f * k1v1.x + 0.15875964f * k2v1.x),
          body1->velocity.y + dt * (0.29697761f * k1v1.y + 0.15875964f * k2v1.y) },
        body1->mass
    };

    Object s2_3 = {
        { body2->position.x + dt * (0.29697761f * k1p2.x + 0.15875964f * k2p2.x),
          body2->position.y + dt * (0.29697761f * k1p2.y + 0.15875964f * k2p2.y) },
        { body2->velocity.x + dt * (0.29697761f * k1v2.x + 0.15875964f * k2v2.x),
          body2->velocity.y + dt * (0.29697761f * k1v2.y + 0.15875964f * k2v2.y) },
        body2->mass
    };

    Vector2 k3v1 = ComputeAcceleration(&s1_3, &s2_3);
    Vector2 k3p1 = s1_3.velocity;

    Vector2 k3v2 = ComputeAcceleration(&s2_3, &s1_3);
    Vector2 k3p2 = s2_3.velocity;

    Object s1_4 = {
        { body1->position.x + dt * (0.21810040f * k1p1.x - 3.05096516f * k2p1.x + 3.83286476f * k3p1.x),
          body1->position.y + dt * (0.21810040f * k1p1.y - 3.05096516f * k2p1.y + 3.83286476f * k3p1.y) },
        { body1->velocity.x + dt * (0.21810040f * k1v1.x - 3.05096516f * k2v1.x + 3.83286476f * k3v1.x),
          body1->velocity.y + dt * (0.21810040f * k1v1.y - 3.05096516f * k2v1.y + 3.83286476f * k3v1.y) },
        body1->mass
    };

    Object s2_4 = {
        { body2->position.x + dt * (0.21810040f * k1p2.x - 3.05096516f * k2p2.x + 3.83286476f * k3p2.x),
          body2->position.y + dt * (0.21810040f * k1p2.y - 3.05096516f * k2p2.y + 3.83286476f * k3p2.y) },
        { body2->velocity.x + dt * (0.21810040f * k1v2.x - 3.05096516f * k2v2.x + 3.83286476f * k3v2.x),
          body2->velocity.y + dt * (0.21810040f * k1v2.y - 3.05096516f * k2v2.y + 3.83286476f * k3v2.y) },
        body2->mass
    };

    Vector2 k4v1 = ComputeAcceleration(&s1_4, &s2_4);
    Vector2 k4p1 = s1_4.velocity;

    Vector2 k4v2 = ComputeAcceleration(&s2_4, &s1_4);
    Vector2 k4p2 = s2_4.velocity;

    body1->velocity.x += dt * (0.17476028f * k1v1.x - 0.55148066f * k2v1.x + 1.20553560f * k3v1.x + 0.17118478f * k4v1.x);
    body1->velocity.y += dt * (0.17476028f * k1v1.y - 0.55148066f * k2v1.y + 1.20553560f * k3v1.y + 0.17118478f * k4v1.y);
    body1->position.x += dt * (0.17476028f * k1p1.x - 0.55148066f * k2p1.x + 1.20553560f * k3p1.x + 0.17118478f * k4p1.x);
    body1->position.y += dt * (0.17476028f * k1p1.y - 0.55148066f * k2p1.y + 1.20553560f * k3p1.y + 0.17118478f * k4p1.y);

    body2->velocity.x += dt * (0.17476028f * k1v2.x - 0.55148066f * k2v2.x + 1.20553560f * k3v2.x + 0.17118478f * k4v2.x);
    body2->velocity.y += dt * (0.17476028f * k1v2.y - 0.55148066f * k2v2.y + 1.20553560f * k3v2.y + 0.17118478f * k4v2.y);
    body2->position.x += dt * (0.17476028f * k1p2.x - 0.55148066f * k2p2.x + 1.20553560f * k3p2.x + 0.17118478f * k4p2.x);
    body2->position.y += dt * (0.17476028f * k1p2.y - 0.55148066f * k2p2.y + 1.20553560f * k3p2.y + 0.17118478f * k4p2.y);
}

float ComputeKineticEnergy(const Object* body)
{
    const float speedSquared = body->velocity.x * body->velocity.x + body->velocity.y * body->velocity.y;
    return 0.5f * body->mass * speedSquared;
}

float ComputePotentialEnergy(const Object* a, const Object* b)
{
    const Vector2 direction = { b->position.x - a->position.x, b->position.y - a->position.y };
    float distance = sqrtf(direction.x * direction.x + direction.y * direction.y);
    if (distance < 1.0f) distance = 1.0f; // Avoid division by zero
    return -(G * a->mass * b->mass) / distance;
}

void ResetBodies(Object* body1, Object* body2)
{
    const Vector2 centerMass = (Vector2){400, 300};

    *body1 = (Object){ { centerMass.x - 100, centerMass.y }, { 0, 0 }, 10.0f };
    *body2 = (Object){ { centerMass.x + 100, centerMass.y }, { 0, 0 }, 10.0f };

    const float distance1 = sqrtf(powf(body2->position.x - body1->position.x, 2) + powf(body2->position.y - body1->position.y, 2));
    const float orbitalSpeed1 = sqrtf(G * (body1->mass + body2->mass) / distance1);

    body1->velocity = (Vector2){ 0, -orbitalSpeed1 * (body2->mass / (body1->mass + body2->mass)) };
    body2->velocity = (Vector2){ 0, orbitalSpeed1 * (body1->mass / (body1->mass + body2->mass)) };
}

int main()
{
    const int screenWidth = 800;
    const int screenHeight = 600;

    InitWindow(screenWidth, screenHeight, "GABRK");
    SetTargetFPS(60);

    const Vector2 centerMass = (Vector2){400, 300};

    Object body1 = (Object){ { centerMass.x - 100, centerMass.y }, { 0, 0 }, 10.0f };
    Object body2 = (Object){ { centerMass.x + 100, centerMass.y }, { 0, 0 }, 10.0f };

    const float distance1 = sqrtf(powf(body2.position.x - body1.position.x, 2) + powf(body2.position.y - body1.position.y, 2));
    const float orbitalSpeed1 = sqrtf(G * (body1.mass + body2.mass) / distance1);

    body1.velocity = (Vector2){ 0, -orbitalSpeed1 * (body2.mass / (body1.mass + body2.mass)) };
    body2.velocity = (Vector2){ 0, orbitalSpeed1 * (body1.mass / (body1.mass + body2.mass)) };

    while (!WindowShouldClose()) 
    {
        if (IsKeyPressed(KEY_RIGHT)) {
            currentMethod = (Method)((currentMethod + 1) % METHOD_COUNT);
            ResetBodies(&body1, &body2);
        }

        if (IsKeyPressed(KEY_LEFT)) {
            currentMethod = (Method)((currentMethod - 1 + METHOD_COUNT) % METHOD_COUNT);
            ResetBodies(&body1, &body2);
        }

        switch (currentMethod)
        {
            case RK1: UpdateRK1(&body1, &body2, TIME_STEP); break;
            case RK2: UpdateRK2_Midpoint(&body1, &body2, TIME_STEP); break;
            case RK2_Heun: UpdateRK2_Heun(&body1, &body2,TIME_STEP); break;
            case RK2_Ralston: UpdateRK2_Ralston(&body1, &body2, TIME_STEP); break;
            case RK3: UpdateRK3_Classic(&body1, &body2, TIME_STEP); break;
            case RK3_Heun: UpdateRK3_Heun(&body1, &body2, TIME_STEP); break;
            case RK3_Ralston: UpdateRK3_Ralston(&body1, &body2, TIME_STEP); break;
            case RK3_HouwenWray: UpdateRK3_HouwenWray(&body1, &body2, TIME_STEP); break;
            case RK3_Strong_Stability_Preserving: UpdateRK3_Strong_Stability_Preserving(&body1, &body2, TIME_STEP); break;
            case RK4: UpdateRK4(&body1, &body2, TIME_STEP); break;
            case RK4_3_8: UpdateRK4_3_8(&body1, &body2, TIME_STEP); break;
            case RK4_Ralston: UpdateRK4_Ralston(&body1, &body2, TIME_STEP); break;
        }

        const float kineticEnergy = ComputeKineticEnergy(&body1) + ComputeKineticEnergy(&body2);
        const float potentialEnergy = ComputePotentialEnergy(&body1, &body2);
        const float totalEnergy = kineticEnergy + potentialEnergy;

        BeginDrawing();
        ClearBackground(RAYWHITE);

        DrawCircleV(body1.position, 10, RED);
        DrawCircleV(body2.position, 10, BLUE);

        DrawText(TextFormat("Current method: %s", methodNames[currentMethod]), 10, 10, 20, BLACK);
        DrawText(TextFormat("Kinetic Energy: %.3f", kineticEnergy), 10, 80, 20, BLACK);
        DrawText(TextFormat("Potential Energy: %.3f", potentialEnergy), 10, 110, 20, BLACK);
        DrawText(TextFormat("Total Energy: %.3f",totalEnergy), 10, 140, 20, BLACK);

        DrawText("ARROWS to change method", 10, GetScreenHeight() - 45, 20, BLACK);
        DrawText("ESC to quit", 10, GetScreenHeight() - 25, 20, BLACK);
        EndDrawing();
    }

    CloseWindow();
}