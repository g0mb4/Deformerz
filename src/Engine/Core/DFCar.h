#pragma once

#include "LibTerep/TerepCar.h"
#include <raylib.h>

#include <stdbool.h>

typedef struct {
    TerepCar* car;
    Texture2D carTex;
    Vector3* vel;   // NOTE(gmb): it can be pos_old for Verlet
    bool renderPhysics;
} DFCar;

DFCar* DFCar_Load();
void DFCar_Unload(DFCar* dfcar);

void DFCar_Update();
