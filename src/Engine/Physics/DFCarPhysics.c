#include "DFCarPhysics.h"
#include "LibTerep/TerepCar.h"
#include "Engine/Engine.h"
#include <raylib.h>
#include <raymath.h>
#include <assert.h>

extern EngineData Engine;

// TODO(gmb): get this values from TEREP2/DAT
#define GRAVITY            9.81f
#define POINT_MASS         2.0f
#define STIFFNESS_NORM  10000.0f
#define STIFFNESS_SUSP  10000.0f
#define DISSIPATION       20.0f

// TODO(gmb): this is duplicated in DFCarRenderer.c
static inline Vector3 ToVector3(float v[3]) { return (Vector3){v[0], v[1], v[2]}; }

// NOTE(gmb): i think this is in the .DAT file, this function can be removed
void DFCar_InitSprings(DFCar* dfcar)
{
    TerepCar* car = dfcar->car;
    TerepCarPoint* points = car->points;

    for (size_t i = 0; i < car->physSegmentCount; i++) {
        TerepCarPhysSegment* seg = &car->physSegments[i];
        Vector3 pA = ToVector3(points[seg->pointA].pos);
        Vector3 pB = ToVector3(points[seg->pointB].pos);
        dfcar->springs[i].restLegnth = Vector3Distance(pA, pB); // TODO(gmb): from .DAT

        if (seg->type == TEREP_SEGMENT_NORMAL) {
            dfcar->springs[i].stiffness = STIFFNESS_NORM;
        } else {
            dfcar->springs[i].stiffness = STIFFNESS_SUSP;
        }
    }
}

void DFCar_UpdatePhysics()
{
    // TODO(gmb): Update inputs somewhere else
    if (IsKeyPressed(KEY_SPACE)) {
        Engine.physicsRunning = !Engine.physicsRunning;
    }

    if (!Engine.physicsRunning) {
        return;
    }

    DFCar* dfcar = Engine.car;
    assert(dfcar);
    TerepCar* car = dfcar->car;
    assert(car);
    float dt = Engine.dt;

    // TODO(gmb): Update inputs somewhere else
    for (size_t i = 0; i < car->pointCount; i++) {
        float speed = 1.0f;
        TerepCarPoint* point = &car->points[i];

        if (IsKeyDown(KEY_UP)) {
            dfcar->vel[i].z -= speed;
        }

        if (IsKeyDown(KEY_DOWN)) {
            dfcar->vel[i].z += speed;
        }

        if (IsKeyDown(KEY_LEFT)) {
            dfcar->vel[i].x -= speed;
        }

         if (IsKeyDown(KEY_RIGHT)) {
            dfcar->vel[i].x += speed;
        }

    }

    Vector3 forces[TEREP_MAX_POINTS] = {0};

    // apply gravity
    for (size_t i = 0; i < car->pointCount; i++) {
        forces[i].y += -POINT_MASS*GRAVITY;
    }

    // Hooke's law
    for (size_t i = 0; i < car->physSegmentCount; i++) {
        TerepCarPhysSegment* seg = &car->physSegments[i];
        Vector3 pA = ToVector3(car->points[seg->pointA].pos);
        Vector3 pB = ToVector3(car->points[seg->pointB].pos);

        float currentLength = Vector3Distance(pB, pA);
        float restLength = dfcar->springs[i].restLegnth;
        Vector3 norm = Vector3Normalize(Vector3Subtract(pB, pA));
        float springForce = dfcar->springs[i].stiffness * (currentLength - restLength);
        Vector3 springForceVector = Vector3Scale(norm, springForce);

        forces[seg->pointA] = Vector3Add(forces[seg->pointA], springForceVector);
        forces[seg->pointB] = Vector3Subtract(forces[seg->pointB], springForceVector);
    }

    // damping
    for (size_t i = 0; i < car->pointCount; i++) {
        forces[i] = Vector3Subtract(forces[i], Vector3Scale(dfcar->vel[i], DISSIPATION));
    }

    // NOTE(gmb): using Explicit-Euler integration for now, but we can switch to Verlet if needed
    for (size_t i = 0; i < car->pointCount; i++) {
        Vector3 acc = Vector3Scale(forces[i], 1.0f/POINT_MASS);
        
        Engine.car->vel[i] = Vector3Add(dfcar->vel[i], Vector3Scale(acc, dt));
        Vector3 pos = Vector3Add(ToVector3(car->points[i].pos), Vector3Scale(dfcar->vel[i], dt));

        car->points[i].pos[0] = pos.x;
        car->points[i].pos[1] = pos.y;
        car->points[i].pos[2] = pos.z;
    }

    // resolve collisions with the map
    for (size_t i = 0; i < car->pointCount; i++) {
        TerepCarPoint* point = &car->points[i];
        float mapHeight = DFMap_GetHeightAt(Engine.map, point->pos[0], point->pos[2]);

        if (point->type == TEREP_POINT_GEOMETRY) {
            if (point->pos[1] < mapHeight) {
                point->pos[1] = mapHeight;
            }
        }

        if ((point->type == TEREP_POINT_WHEEL_FRONT || point->type == TEREP_POINT_WHEEL_REAR)) {
            if (point->pos[1] - point->size < mapHeight) {
                point->pos[1] = mapHeight + point->size;
            }
        }
    }
}