#include "DFCarPhysics.h"
#include "Engine/Engine.h"
#include "LibTerep/TerepCar.h"
#include <assert.h>
#include <raylib.h>
#include <raymath.h>

extern EngineData Engine;

// DIRECT ASSEMBLY CONSTANTS (In Native Float Units)
// Gravity subtracted directly from Y-velocity per tick
#define GRAVITY 0.25f // Scaled for world units (Assembly: 0x0020 / 128)

// Assembly bit-shifts: sar ax, 5 (divide by 32) and sar ax, 3 (divide by 8)
#define STIFFNESS (1.0f / 32.0f)  // 0.03125f
#define DISSIPATION (1.0f / 8.0f) // 0.125f

static inline Vector3 ToVector3(float v[3]) { return (Vector3){v[0], v[1], v[2]}; }

void DFCar_UpdatePhysics()
{
    DFCar* dfcar = Engine.car;
    assert(dfcar);
    TerepCar* car = dfcar->car;
    assert(car);

    if (IsKeyPressed(KEY_SPACE)) {
        Engine.physicsRunning = !Engine.physicsRunning;
    }

    if (!Engine.physicsRunning)
        return;

    // 1. Cache Map Heights
    for (size_t i = 0; i < car->pointCount; i++) {
        TerepCarPoint* point = &car->points[i];
        dfcar->mapHeights[i] = DFMap_GetHeightAt(Engine.map, point->pos[0], point->pos[2]);
    }

    Vector3 forces[TEREP_MAX_POINTS] = {0};

    // 2. Apply Assembly Gravity
    for (size_t i = 0; i < car->pointCount; i++) {
        forces[i].y -= GRAVITY;
    }

    // 3. Mass-Spring Segment Loop
    for (size_t i = 0; i < car->physSegmentCount; i++) {
        TerepCarPhysSegment* seg = &car->physSegments[i];

        Vector3 pA = ToVector3(car->points[seg->pointA].pos);
        Vector3 pB = ToVector3(car->points[seg->pointB].pos);

        Vector3 delta = Vector3Subtract(pB, pA);
        float currentLength = Vector3Length(delta);
        if (currentLength < 0.0001f)
            continue;

        Vector3 norm = Vector3Scale(delta, 1.0f / currentLength);
        float restLength = seg->length1;
        float displacement = currentLength - restLength;

        // Relative velocity along link axis
        Vector3 velA = dfcar->vel[seg->pointA];
        Vector3 velB = dfcar->vel[seg->pointB];
        Vector3 relVel = Vector3Subtract(velB, velA);

        // Assembly bit-shifts: sar ax, 5 and sar ax, 3
        float springForce = displacement * STIFFNESS;
        float dampingForce = Vector3DotProduct(relVel, norm) * DISSIPATION;

        float totalForce = springForce + dampingForce;
        Vector3 forceVector = Vector3Scale(norm, totalForce);

        // Equal and opposite reactions:
        // Stretched (totalForce > 0): A pulled TOWARD B (+), B pulled TOWARD A (-)
        forces[seg->pointA] = Vector3Add(forces[seg->pointA], forceVector);
        forces[seg->pointB] = Vector3Subtract(forces[seg->pointB], forceVector);
    }

    // 4. Assembly Integration Step (Symplectic Euler)
    for (size_t i = 0; i < car->pointCount; i++) {
        // v = v + force
        dfcar->vel[i] = Vector3Add(dfcar->vel[i], forces[i]);

        // x = x + v (Pos and Vel share same units)
        car->points[i].pos[0] += dfcar->vel[i].x / 4.0;
        car->points[i].pos[1] += dfcar->vel[i].y / 4.0;
        car->points[i].pos[2] += dfcar->vel[i].z / 4.0;
    }

    // 5. Ground Collision Resolution
    for (size_t i = 0; i < car->pointCount; i++) {
        TerepCarPoint* point = &car->points[i];
        float mapHeight = dfcar->mapHeights[i];
        float minHeight = mapHeight;

        if (point->type == TEREP_POINT_WHEEL_FRONT || point->type == TEREP_POINT_WHEEL_REAR) {
            minHeight += point->size;
        }

        if (point->pos[1] < minHeight) {
            point->pos[1] = minHeight;
            if (dfcar->vel[i].y < 0.0f) {
                dfcar->vel[i].y = 0.0f;
            }
        }
    }
}