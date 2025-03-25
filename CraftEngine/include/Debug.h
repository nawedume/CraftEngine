#pragma once

#include "Core.h"
#include <string>

/**
* Quick and easy way of inserting draw calls in the non-user facing code.
*/
namespace ce {

    extern bool isOn;

    typedef void (*drawDebugBall)(Vec3);
    typedef void (*drawDebugBallWithOri)(Vec3, Quat q);
    typedef void (*drawDebugBallWithColor)(Vec3, Vec3 q);
    typedef void (*drawDebugArrow)(Vec3 pos, Vec3 dir);

    extern void PrintVec3(std::string prefix, Vec3 v);

    extern void SetDrawDebugBallFn(drawDebugBall callback);

    extern void SetDrawDebugBallFn(drawDebugBallWithOri callback);

    extern void SetDrawDebugBallFn(drawDebugBallWithColor callback);

    extern void SetDrawArrowDebugFn(drawDebugArrow callback);

    extern void DrawDebugBall(Vec3 pos);

    extern void DrawDebugBall(Vec3 pos, Quat q);

    extern void DrawDebugBall(Vec3 pos, Vec3 color);

    extern void DrawDebugArrow(Vec3 pos, Vec3 normal);
}
