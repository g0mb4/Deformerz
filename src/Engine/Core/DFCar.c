#include "Engine/Engine.h"
#include "DFCar.h"
#include "LibTerep/TerepCar.h"
#include <raylib.h>
#include <raymath.h>

#include <assert.h>
#include <stdlib.h>

extern EngineData Engine;

#define GRAVITY 9.81f   // TODO(gmb): get this value from TEREP2

// TODO(gmb): this is duplicated in DFCarRenderer.c
static inline Vector3 ToVector3(float v[3]) { return (Vector3){v[0], v[1], v[2]}; }

// NOTE(gmb): i think this is in the .DAT file
void init_springs(DFCar* dfcar)
{ 
    TerepCar* car = dfcar->car;
    TerepCarPoint* points = car->points;

    for (size_t i = 0; i < car->physSegmentCount; i++) {
        TerepCarPhysSegment* seg = &car->physSegments[i];
        Vector3 pA = ToVector3(points[seg->pointA].pos);
        Vector3 pB = ToVector3(points[seg->pointB].pos);
        dfcar->springs[i].restLegnth = Vector3Distance(pA, pB); // TODO(gmb): from .DAT
        dfcar->springs[i].stiffness = 1500.0f;                  // TODO(gmb): from .DAT
    }
}

DFCar* DFCar_Load()
{
    DFCar* dfcar = calloc(1, sizeof(DFCar));
    assert(dfcar);
    dfcar->car = TerepCar_Load("./data/car1.dat", "./data/car1.pcx");
    dfcar->carTex = LoadTextureFromImage((Image){
        .data = dfcar->car->carTexture->data,
        .height = dfcar->car->carTexture->height,
        .width = dfcar->car->carTexture->width,
        .format = 7,
        .mipmaps = 1,
    });
    dfcar->vel = calloc(dfcar->car->pointCount, sizeof(Vector3));
    assert(dfcar->vel);
    dfcar->springs = calloc(dfcar->car->physSegmentCount, sizeof(Spring));
    assert(dfcar->springs);
    init_springs(dfcar);
    return dfcar;
}
void DFCar_Unload(DFCar* dfcar)
{
    TerepCar_Unload(dfcar->car);
    UnloadTexture(dfcar->carTex);
    free(dfcar);
}
void DFCar_Update()
{ 
    // TODO(gmb): Update inputs somewhere else
    if (IsKeyPressed(KEY_SPACE)) {
        Engine.physicsRunning = !Engine.physicsRunning;
    }
    if (!Engine.physicsRunning) return;

    DFCar* dfcar = Engine.car;
    assert(dfcar);
    TerepCar* car = dfcar->car;
    assert(car);
    float dt = Engine.dt;

    Vector3 forces[TEREP_MAX_POINTS] = { 0 };

    // apply gravity
    for (size_t i = 0; i < car->pointCount; i++) {
        forces[i].y += -GRAVITY;
    }

    // Hooke's law
    // TODO(gmb): damping ?
    for (size_t i = 0; i < car->physSegmentCount; i++) {
        TerepCarPhysSegment* seg = &car->physSegments[i];
        Vector3 pA = ToVector3(car->points[seg->pointA].pos);
        Vector3 pB = ToVector3(car->points[seg->pointB].pos);

        float currentLength = Vector3Distance(pA, pB);
        float restLength = dfcar->springs[i].restLegnth;
        Vector3 norm = Vector3Normalize(Vector3Subtract(pB, pA));
        Vector3 springForce = Vector3Scale(norm, dfcar->springs[i].stiffness * (currentLength - restLength));
        forces[seg->pointA] = Vector3Add(forces[seg->pointA], springForce);
        forces[seg->pointB] = Vector3Subtract(forces[seg->pointB], springForce);
    }

    // resolve collisions with the map
    for (size_t i = 0; i < car->pointCount; i++) {
        TerepCarPoint* point = &car->points[i];
        if (point->type == TEREP_POINT_GEOMETRY) {
            // TODO(gmb): resolve point collision here
        } else if (point->type == TEREP_POINT_WHEEL_FRONT || point->type == TEREP_POINT_WHEEL_REAR) {
            // TODO(gmb): resolve shpere collision here
        } else {
            // DO NOTHING
        }
    }

    // NOTE(gmb): using Explicit-Euler integration for now, but we can switch to Verlet if needed
    for (size_t i = 0; i < car->pointCount; i++) {
        // NOTE(gmb): mass is assumed to be 1
        Engine.car->vel[i] = Vector3Add(dfcar->vel[i], Vector3Scale(forces[i], dt));
        Vector3 pos = Vector3Add(ToVector3(car->points[i].pos), Vector3Scale(dfcar->vel[i], dt));

        car->points[i].pos[0] = pos.x;
        car->points[i].pos[1] = pos.y;
        car->points[i].pos[2] = pos.z;
    }

    // clamp wheels
    for (size_t i = 0; i < car->pointCount; i++) {
        TerepCarPoint* point = &car->points[i];
        if (point->type == TEREP_POINT_WHEEL_FRONT || point->type == TEREP_POINT_WHEEL_REAR) {
            Vector3 pos = ToVector3(point->pos);
            float mapHeight = -0.1; // TODO(gmb): get it from heightmap
            if (pos.y < mapHeight) {
                point->pos[1] = mapHeight;
            }
        }
    }
}
