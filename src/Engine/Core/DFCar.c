#include "Engine/Engine.h"
#include "DFCar.h"
#include "LibTerep/TerepCar.h"
#include "Engine/Physics/DFCarPhysics.h"

#include <assert.h>
#include <stdlib.h>

// TODO(gmb): this is duplicated in DFCarRenderer.c
static inline Vector3 ToVector3(float v[3]) { return (Vector3){v[0], v[1], v[2]}; }

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
    dfcar->mapHeights = calloc(dfcar->car->pointCount, sizeof(float));
    assert(dfcar->mapHeights);
    DFCar_InitSprings(dfcar);
    return dfcar;
}
void DFCar_Unload(DFCar* dfcar)
{
    TerepCar_Unload(dfcar->car);
    UnloadTexture(dfcar->carTex);
    free(dfcar);
}
