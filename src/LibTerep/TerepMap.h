#pragma once

#include "PCX.h"
#include <stdint.h>

#define TMAP_SCALE 1.0f
#define TMAP_HEIGHT_SCALE 0.075f
#define TMAP_UVMULT 0.0625f

#define TEREP_MAPSZ 256
#define TEREP_TEXSZ 256

typedef struct {
    int vertexCount;
    int triangleCount;
    float* vertices;
    float* uvs;
    PCXData* heightmap;
    PCXData* colormap;
    PCXImage* texturemap;
} TerepMap;

TerepMap* TerepMap_Load(const char* colpcx, const char* mappcx, const char* maptexpcx);
void TerepMap_Unload(TerepMap* map);
