#include "DFMap.h"
#include "Engine/Rendering/Shaders.h"
#include "Engine/Engine.h"

#include <assert.h>
#include <stdlib.h>

DFMap* DFMap_Load()
{
    DFMap* dfmap = calloc(1, sizeof(DFMap));
    assert(dfmap);
    dfmap->map = TerepMap_Load("./data/col.pcx", "./data/map.pcx", "./data/maptex.pcx");
    for (size_t i = 0; i < 256; i++)
    {
        Engine.palette[i].r = PCX_GLOBAL_PALETTE[i].red;
        Engine.palette[i].g = PCX_GLOBAL_PALETTE[i].green;
        Engine.palette[i].b = PCX_GLOBAL_PALETTE[i].blue;
        Engine.palette[i].a = 255;
    }
    Engine.skyColor = Engine.palette[255];
    dfmap->tex = LoadTextureFromImage((Image){
        .data = dfmap->map->texturemap->data,
        .height = dfmap->map->texturemap->height,
        .width = dfmap->map->texturemap->width,
        .format = 7,
        .mipmaps = 1,
    });
    Mesh msh = {0};
    msh.triangleCount = dfmap->map->triangleCount;
    msh.vertexCount = dfmap->map->vertexCount;
    msh.vertices = dfmap->map->vertices;
    msh.texcoords = dfmap->map->uvs;
    UploadMesh(&msh, false);
    dfmap->model = LoadModelFromMesh(msh);
    dfmap->shader = LoadShaderFromMemory(Affine_vs, Affine_fs);
    dfmap->model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = dfmap->tex;
    return dfmap;
}
void DFMap_Unload(DFMap* dfmap)
{
    TerepMap_Unload(dfmap->map);
    UnloadTexture(dfmap->tex);
    UnloadModel(dfmap->model);
    UnloadShader(dfmap->shader);
    free(dfmap);
}
float DFMap_GetHeightAt(DFMap* dfmap, float x, float z) 
{ 
    float xInMapSpace = (x / TMAP_SCALE) + (TEREP_MAPSZ / 2.0f);
    float zInMapSpace = (z / TMAP_SCALE) + (TEREP_MAPSZ / 2.0f);
        
    int mx = (int)xInMapSpace;
    int mz = (int)zInMapSpace;

    // NOTE(gmb): will fail on the edges, solve it later
    assert(mx >= 1 && mx < TEREP_MAPSZ-1);
    assert(mz >= 1 && mz < TEREP_MAPSZ-1);

    float dx = xInMapSpace - mx;
    float dz = zInMapSpace - mz;

    float hx0z0 = dfmap->map->heightmap->data[mx + mz * TEREP_MAPSZ] * TMAP_HEIGHT_SCALE;
    float hx1z0 = dfmap->map->heightmap->data[(mx + 1) + mz * TEREP_MAPSZ] * TMAP_HEIGHT_SCALE;
    float hx0z1 = dfmap->map->heightmap->data[mx + (mz + 1) * TEREP_MAPSZ] * TMAP_HEIGHT_SCALE;
    float hx1z1 = dfmap->map->heightmap->data[(mx + 1) + (mz + 1) * TEREP_MAPSZ] * TMAP_HEIGHT_SCALE;
        
    float height = 0;
    if (dx + dz <= 1.0f) {
        height = hx0z0 + dx * (hx1z0 - hx0z0) + dz * (hx0z1 - hx0z0);
    } else {
        height = hx1z1 * (1.0f - dx) * (hx0z1 - hx1z1) + (1.0f - dz) * (hx1z0 - hx1z1);
    }

    // NOTE(gmb): -5.0f comes from Renderer_RenderMap()
    return height - 5.0f;
}
