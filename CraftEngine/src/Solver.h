#pragma once

#include "World.h"

namespace ce {

    /*
     * @breif Fills in needed values for each manifold.
     */
    void PrepManifolds(WorldContactSet* contactSet, RigidBody* bodies, Material* materials, ImpulseStore* impulseStore, Real inverseDt, WorldSettings settings);

    void SolveManifolds(RigidBody* bodies, WorldContactSet* contactSet, Real inverseDt);
}
