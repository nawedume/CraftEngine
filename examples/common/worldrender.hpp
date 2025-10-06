#include "Drawing.h"
#include "World.h"

inline void DrawWorld(draw::GObject *objects, ce::World *world, draw::GSystem *gsys) {
    for (int i = 0; i < world->NumBodies(); ++i) {
        auto gobj = objects[i];
        auto t = world->Transforms[i];
        auto collider = world->Colliders[i];
        switch (collider.Id) {
        case ce::ColliderId::SPHERE:
            draw::DrawBall(&gobj, &gsys->mShader, t.Pos.x, t.Pos.y, t.Pos.z, t.Orientation.w, t.Orientation.x,
                           t.Orientation.y, t.Orientation.z);
            break;
        case ce::ColliderId::HULL:
            draw::DrawBox(&gobj, &gsys->mShader, t.Pos.x, t.Pos.y, t.Pos.z, t.Orientation.w, t.Orientation.x,
                          t.Orientation.y, t.Orientation.z);
            break;
        case ce::ColliderId::CAPSULE:
            draw::DrawObject(&gobj, &gsys->mShader, t.Pos.x, t.Pos.y, t.Pos.z, t.Orientation.w, t.Orientation.x,
                           t.Orientation.y, t.Orientation.z);
            break;
        }
    }
}

inline void DrawWorldRelative(draw::GObject *objects, ce::World *world, draw::GSystem *gsys,
                              ce::Transform *worldToLocalTransform) {
    for (int i = 0; i < world->NumBodies(); ++i) {
        auto gobj = objects[i];
        auto t = worldToLocalTransform->Compose(world->Transforms[i]);
        auto collider = world->Colliders[i];

        switch (collider.Id) {
        case ce::ColliderId::SPHERE:
            draw::DrawBall(&gobj, &gsys->mShader, t.Pos.x, t.Pos.y, t.Pos.z, t.Orientation.w, t.Orientation.x,
                           t.Orientation.y, t.Orientation.z);
            break;
        case ce::ColliderId::HULL:
            draw::DrawBox(&gobj, &gsys->mShader, t.Pos.x, t.Pos.y, t.Pos.z, t.Orientation.w, t.Orientation.x,
                          t.Orientation.y, t.Orientation.z);
            break;
        }
    }
}
