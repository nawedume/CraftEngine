#include "Core.h"
#include "World.h"
#include "editor.h"
#include "ex_boxstacking.h"

void ExBoxStackingSetup(ceeditor::Editor* editor) {
    editor->World->Settings.NumOfSolverIterations = 10;
    editor->World->Settings.NumOfRelaxationIterations = 0;

    ce::BoxDef floorDef = ce::BoxDef {
        .HalfEdge = {10.0, 1.0, 10.0},
    };
    ce::BodyId floor = editor->AddBox(floorDef, {}, ce::Vec3(0.4, 0.4, 0.4));
    ce::RigidBody& floorB = editor->World->RigidBodies[floor];
    ce::RigidBodyBase& floorB1 = editor->World->RigidBodiesBase[floor];
    floorB.InverseMass = 0.0;
    floorB.InverseInertia = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    floorB1.InverseInertiaLocal = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    ce::BoxDef boxDef = ce::BoxDef{.Mass = .1f, .HalfEdge = {2.0, 1.0, 2.0}};
    const int NUM_BOXES = 10;
    for (int i = 0; i < NUM_BOXES; ++i) {
        editor->AddBox(boxDef, ce::Transform{.Pos = {0.0f, 2.0f + (i * 2.1), 0.0f}}, ce::Vec3(0.5, 0.1, 0.1));
    }
}
