#pragma once

#include "LibTerep/TerepCar.h"
#include <raylib.h>

#include <stdbool.h>

// TODO(gmb): this must be part of TerepCar
typedef struct {
    float restLegnth;
    float stiffness;
} Spring;

typedef struct {
    TerepCar* car;
    Texture2D carTex;
    Vector3* vel;       // NOTE(gmb): it can be pos_old for Verlet
    Spring* springs;    // NOTE(gmb): these should be come from the .DAT file
    float* mapHeights;  // NOTE(gmb): projected height on the map along Y, TerepCar structure?
    bool renderPhysics;
} DFCar;

DFCar* DFCar_Load();
void DFCar_Unload(DFCar* dfcar);
