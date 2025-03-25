#pragma once

#include "World.h"

namespace ce {

    extern void BruteForceBroadPhase(World* world);

    extern void BruteForceBroadPhaseSIMD(World* world);
}
