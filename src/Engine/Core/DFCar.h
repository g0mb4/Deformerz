#pragma once

#include "LibTerep/TerepCar.h"
#include <raylib.h>

#include <stdbool.h>

typedef struct {
    float restLegnth;
    float stiffness;
} Spring;

typedef struct {
    TerepCar* car;
    Texture2D carTex;
    Vector3* vel;       // NOTE(gmb): it can be pos_old for Verlet
    Spring* springs;    // NOTE(gmb): these should be come from the .DAT file
    bool renderPhysics;
} DFCar;

DFCar* DFCar_Load();
void DFCar_Unload(DFCar* dfcar);

void DFCar_Update();
