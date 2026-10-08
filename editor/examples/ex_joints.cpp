#include "Math.hpp"
#include "World.h"
#include "ex_joints.h"

#include "imgui.h"

void ExJointsSetup(ceeditor::Editor *editor) {
    editor->World->Settings.NumOfSolverIterations = 10;
    editor->World->Settings.NumOfRelaxationIterations = 0;

    ce::BoxDef boxDef = ce::BoxDef{.Mass = .1f, .HalfEdge={2.0, 2.0, 2.0}};

    ce::BodyId topBox = editor->AddBox(
        boxDef,
        ce::Transform{.Pos = {0.0f, (11 * 6.0f), 0.0f}},
        ce::Vec3(0.5, 0.1, 0.1));

    ce::RigidBody& boxrb = editor->World->RigidBodies[topBox];
    boxrb.InverseMass = 0.0;
    boxrb.InverseInertia = ce::ZERO_MAT;
    ce::RigidBodyBase& boxrb1 = editor->World->RigidBodiesBase[topBox];
    boxrb1.InverseInertiaLocal = ce::ZERO_MAT;

    ce::BodyId prevBox = topBox;
    const int NUM_BOXES = 10;
    for (int i = 0; i < NUM_BOXES; ++i) {
        ce::Vec3 pos = {0.0f, ((10 - i) * 6.0f), 0.0f};
            auto currentBox = editor->AddBox(
                boxDef, ce::Transform{.Pos = pos }, ce::Vec3(0.5, 0.1, 0.1)
            );

        ce::AddRevoluteJoint(editor->World, prevBox, currentBox, pos + ce::Vec3(0.0f, 3.0f, 0.0f));
        prevBox = currentBox;
    }
}

void ExJointsUISetup(ceeditor::Editor* editor) {
    ImGui::Spacing();
    ImGui::Text("Joint Example Actions");
    ImGui::Button("Add Force");
    if (ImGui::IsItemActive()) {
        ce::ApplyLinearForce(editor->World, 10, ce::Vec3(10.0f, 0.0, 0.0));
    }
}
