#include "BroadPhase.h"
#include "AABB.h"
#include "World.h"

namespace ce {

    void BruteForceBroadPhase(World* world) {
        world->BroadPhaseBodies.clear();
        for (u32 i = 0; i < world->NumBodies() ; ++i) {
            AABB aabb = world->AABBs[i];
            for (u32 j = i + 1; j < world->NumBodies(); ++j) {
                bool doesOverlap = AABBOverlapTest(&aabb, &world->AABBs[j]);
                if (doesOverlap) {
                    world->BroadPhaseBodies.push_back(BodyPair { i, j });
                }
            }
        }
    }
}
