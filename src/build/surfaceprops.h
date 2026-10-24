// Titanfall 2 surface properties, in the order of scripts/surfaceproperties.txt
// (the index is stored per triangle in the per-tri AABB block) with densities in kg/m^3.
#pragma once

#include <string>

#include "core/strings.h"

namespace r2c {

struct SurfaceProp {
    const char* name;
    float density;
};

inline const SurfaceProp kSurfaceProps[] = {
    {"default", 2000}, {"reflective", 2700}, {"weightedcube_bounce", 2700}, {"paintbomb", 2700}, {"sphere2", 1000},
    {"metal_titan", 2700}, {"solidmetal", 2700}, {"metalgrate", 1600}, {"metalvent", 2700}, {"dirt", 1600},
    {"tile", 2700}, {"xo_shield", 2000}, {"wood", 700}, {"wood_solid", 700}, {"water", 1000}, {"concrete", 2400},
    {"glass", 2700}, {"glass_breakable", 2700}, {"flesh", 980}, {"armorflesh", 980}, {"sand", 1600}, {"mud", 1600},
    {"metal", 2700}, {"grass", 1600}, {"brokenglass", 2700}, {"gravel", 2400}, {"bloodyflesh", 980},
    {"shellcasing_small", 2700}, {"shellcasing_large", 2700}, {"weapon", 2700}, {"computer", 2700}, {"canister", 2700},
    {"metal_barrel", 2700}, {"glassbottle", 700}, {"pottery", 700}, {"grenade", 2700}, {"grenade_triple_threat", 2700},
    {"bouncygrenade", 2000}, {"metal_box", 2700}, {"metal_bouncy", 10000}, {"slipperymetal", 2700},
    {"metalpanel", 2700}, {"slipperyslime", 1600}, {"wood_lowdensity", 300}, {"wood_box", 700}, {"wood_crate", 700},
    {"wood_plank", 700}, {"wood_furniture", 700}, {"wood_panel", 700}, {"slime", 2000}, {"quicksand", 600},
    {"rock", 2400}, {"porcelain", 2400}, {"boulder", 2400}, {"asphalt", 2400}, {"brick", 2400},
    {"concrete_block", 2400}, {"chainlink", 1600}, {"chain", 1600}, {"alienflesh", 980}, {"watermelon", 900},
    {"snow", 800}, {"ice", 917}, {"carpet", 500}, {"unused_slot_to_prevent_renumbering", 2000}, {"plaster", 2400},
    {"cardboard", 500}, {"plastic_barrel", 500}, {"plastic_box", 500}, {"plastic", 500}, {"item", 600},
    {"floatingstandable", 800}, {"rubber", 500}, {"rubbertire", 500}, {"jeeptire", 500}, {"slidingrubbertire", 500},
    {"brakingrubbertire", 500}, {"slidingrubbertire_front", 500}, {"slidingrubbertire_rear", 500},
    {"floating_metal_barrel", 500}, {"plastic_barrel_buoyant", 150}, {"roller", 2700}, {"popcan", 700},
    {"paintcan", 700}, {"paper", 500}, {"papercup", 500}, {"ceiling_tile", 2400}, {"default_silent", 2000},
    {"player", 1000}, {"player_control_clip", 2000}, {"no_decal", 900}, {"foliage", 700}, {"underground_cube", 2700},
    {"turret_gib", 2700}, {"metal_spectre", 2700}, {"arc_grenade", 2700}, {"upholstery", 500}, {"flyerflesh", 980},
};

// Index in surfaceproperties.txt order, or -1.
inline int SurfacePropIndex(const std::string& name) {
    int i = 0;
    for (auto& s : kSurfaceProps) {
        if (IEquals(name, s.name)) return i;
        i++;
    }
    return -1;
}

}  // namespace r2c
