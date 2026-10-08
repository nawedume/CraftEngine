#include "Core.h"
#include "World.h"
#include "Math.hpp"

#include "glm/gtc/constants.hpp"
#include <vector>

#include "common/utils.hpp"
#include "ex_bowling.h"

void ExBowlingSetup(ceeditor::Editor *editor) {
    ce::BoxDef floorDef = ce::BoxDef{ .HalfEdge = ce::Vec3(10.0f, 1.0f, 10.0f)};
    ce::BodyId floor = editor->AddBox(floorDef, ce::Transform{}, ce::Vec3(0.4, 0.4, 0.4));
    ce::SetStatic(editor->World, floor);

    ce::Quat ori = ce::aaToQuat(ce::Vec3(1.0, 0.0, 0.0), glm::quarter_pi<float>());
    ce::Vec3 p = ce::Vec3(0.0f, 10.f, -18.0f);
    ce::BodyId ramp = editor->AddBox(floorDef, ce::Transform{ .Pos = p, .Orientation = ori }, ce::Vec3(0.4, 0.4, 0.4));
    ce::SetStatic(editor->World, ramp);

    ce::CapsuleDef capsuleDef = ce::CapsuleDef{.Mass = 100.0f, .Radius = 1.0f, .HalfLength = 1.0f};
    ce::BodyId capsule = editor->AddCapsule(
        capsuleDef, ce::Transform { .Pos = { 0.0f, 20.0f, -20.0f } }, ce::Vec3(0.7f, 0.0f, 0.0f)
    );
    editor->World->Materials[capsule].Friction = 0.2;

    ce::BodyId pin0 = editor->AddCapsule(capsuleDef, ce::Transform { .Pos = {  0.0f, 3.0f, 0.0f } }, ce::Vec3(0.7f, 0.0f, 0.0f));
    ce::BodyId pin1 = editor->AddCapsule(capsuleDef, ce::Transform { .Pos = {  1.0f, 3.0f, 2.1f } }, ce::Vec3(0.7f, 0.0f, 0.0f));
    ce::BodyId pin2 = editor->AddCapsule(capsuleDef, ce::Transform { .Pos = { -1.0f, 3.0f, 2.1f } }, ce::Vec3(0.7f, 0.0f, 0.0f));
    ce::BodyId pin3 = editor->AddCapsule(capsuleDef, ce::Transform { .Pos = { -2.0f, 3.0f, 4.2f } }, ce::Vec3(0.7f, 0.0f, 0.0f));
    ce::BodyId pin4 = editor->AddCapsule(capsuleDef, ce::Transform { .Pos = {  0.0f, 3.0f, 4.2f } }, ce::Vec3(0.7f, 0.0f, 0.0f));
    ce::BodyId pin5 = editor->AddCapsule(capsuleDef, ce::Transform { .Pos = {  2.0f, 3.0f, 4.2f } }, ce::Vec3(0.7f, 0.0f, 0.0f));
}
