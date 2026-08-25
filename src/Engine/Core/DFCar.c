#include "Engine/Engine.h"
#include "DFCar.h"
#include "LibTerep/TerepCar.h"
#include <raylib.h>

#include <assert.h>
#include <stdlib.h>

extern EngineData Engine;

#define GRAVITY 9.81f

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
    return dfcar;
}
void DFCar_Unload(DFCar* dfcar)
{
    TerepCar_Unload(dfcar->car);
    UnloadTexture(dfcar->carTex);
    free(dfcar);
}
static void update_gravity(size_t id, float dt)
{
    // NOTE(gmb): using Explicit-Euler integration for now, but we can switch to Verlet if needed
    Engine.car->vel[id].y += -GRAVITY * dt;
    Engine.car->car->points[id].pos[1] += Engine.car->vel[id].y * dt;
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

    for (size_t i = 0; i < car->pointCount; i++) {
        update_gravity(i, dt);
    }
}
