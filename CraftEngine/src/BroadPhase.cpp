#include "BroadPhase.h"
#include "AABB.h"
#include "World.h"
#include <simde/x86/sse2.h>

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

    // Helper function: Compute the absolute value of a simde__m128 using a bit-mask.
    inline simde__m128 abs_ps(simde__m128 x) {
        const simde__m128 mask = simde_mm_castsi128_ps(simde_mm_set1_epi32(0x7fffffff));
        return simde_mm_and_ps(x, mask);
    }

    // SIMD-enabled brute force broad-phase
    // Not really a smart of way of doing this. TODO: Try implementing SAP or BVH to handle this
    void BruteForceBroadPhaseSIMD(World* world) {
        world->BroadPhaseBodies.clear();
        unsigned n = world->NumBodies();

        // Loop over each AABB in the world.
        for (unsigned i = 0; i < n; ++i) {
            // Load the i-th AABB’s center and half-edge values.
            float cx = world->AABBs[i].Center.x;
            float cy = world->AABBs[i].Center.y;
            float cz = world->AABBs[i].Center.z;
            float hx = world->AABBs[i].HalfEdge.x;
            float hy = world->AABBs[i].HalfEdge.y;
            float hz = world->AABBs[i].HalfEdge.z;

            // Replicate these values into SIMD registers.
            simde__m128 v_cx = simde_mm_set1_ps(cx);
            simde__m128 v_cy = simde_mm_set1_ps(cy);
            simde__m128 v_cz = simde_mm_set1_ps(cz);
            simde__m128 v_hx = simde_mm_set1_ps(hx);
            simde__m128 v_hy = simde_mm_set1_ps(hy);
            simde__m128 v_hz = simde_mm_set1_ps(hz);

            // Process potential overlaps in blocks of 4.
            unsigned j = i + 1;
            for (; j + 3 < n; j += 4) {
                // Load center components for bodies j, j+1, j+2, j+3.
                simde__m128 other_cx = simde_mm_set_ps(
                    world->AABBs[j+3].Center.x,
                    world->AABBs[j+2].Center.x,
                    world->AABBs[j+1].Center.x,
                    world->AABBs[j].Center.x
                );
                simde__m128 other_cy = simde_mm_set_ps(
                    world->AABBs[j+3].Center.y,
                    world->AABBs[j+2].Center.y,
                    world->AABBs[j+1].Center.y,
                    world->AABBs[j].Center.y
                );
                simde__m128 other_cz = simde_mm_set_ps(
                    world->AABBs[j+3].Center.z,
                    world->AABBs[j+2].Center.z,
                    world->AABBs[j+1].Center.z,
                    world->AABBs[j].Center.z
                );
                // Load half-edge components.
                simde__m128 other_hx = simde_mm_set_ps(
                    world->AABBs[j+3].HalfEdge.x,
                    world->AABBs[j+2].HalfEdge.x,
                    world->AABBs[j+1].HalfEdge.x,
                    world->AABBs[j].HalfEdge.x
                );
                simde__m128 other_hy = simde_mm_set_ps(
                    world->AABBs[j+3].HalfEdge.y,
                    world->AABBs[j+2].HalfEdge.y,
                    world->AABBs[j+1].HalfEdge.y,
                    world->AABBs[j].HalfEdge.y
                );
                simde__m128 other_hz = simde_mm_set_ps(
                    world->AABBs[j+3].HalfEdge.z,
                    world->AABBs[j+2].HalfEdge.z,
                    world->AABBs[j+1].HalfEdge.z,
                    world->AABBs[j].HalfEdge.z
                );

                // Compute the absolute differences between the centers.
                simde__m128 diff_x = abs_ps(simde_mm_sub_ps(v_cx, other_cx));
                simde__m128 diff_y = abs_ps(simde_mm_sub_ps(v_cy, other_cy));
                simde__m128 diff_z = abs_ps(simde_mm_sub_ps(v_cz, other_cz));

                // Compute the sum of the half-edges for each axis.
                simde__m128 sum_hx = simde_mm_add_ps(v_hx, other_hx);
                simde__m128 sum_hy = simde_mm_add_ps(v_hy, other_hy);
                simde__m128 sum_hz = simde_mm_add_ps(v_hz, other_hz);

                // Compare differences to the allowed extents.
                simde__m128 cmp_x = simde_mm_cmple_ps(diff_x, sum_hx);
                simde__m128 cmp_y = simde_mm_cmple_ps(diff_y, sum_hy);
                simde__m128 cmp_z = simde_mm_cmple_ps(diff_z, sum_hz);

                // A valid overlap requires all three comparisons to be true.
                simde__m128 overlap = simde_mm_and_ps(cmp_x, simde_mm_and_ps(cmp_y, cmp_z));

                // Create a 4-bit mask from the SIMD comparison.
                int mask = simde_mm_movemask_ps(overlap);
                if (mask != 0) {
                    // For each lane that passed the test, record the overlapping pair.
                    for (int lane = 0; lane < 4; ++lane) {
                        if (mask & (1 << lane)) {
                            world->BroadPhaseBodies.push_back({ i, j + lane });
                        }
                    }
                }
            }

            // Process any remaining bodies that were not handled in the SIMD loop.
            for (; j < n; ++j) {
                float dx = std::fabs(cx - world->AABBs[j].Center.x);
                float dy = std::fabs(cy - world->AABBs[j].Center.y);
                float dz = std::fabs(cz - world->AABBs[j].Center.z);
                if (dx <= (hx + world->AABBs[j].HalfEdge.x) &&
                    dy <= (hy + world->AABBs[j].HalfEdge.y) &&
                    dz <= (hz + world->AABBs[j].HalfEdge.z)) {
                    world->BroadPhaseBodies.push_back({ i, j });
                }
            }
        }
    }
}
