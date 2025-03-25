#include "Debug.h"
#include "Core.h"

namespace ce {

static bool initialized = false;

void PrintVec3(std::string prefix, Vec3 v) { printf("%s: %f, %f, %f\n", prefix.c_str(), v.x, v.y, v.z); }

drawDebugBall DrawDebugBallFn;
drawDebugBallWithOri DrawDebugBallWithOriFn;
drawDebugBallWithColor DrawDebugBallWithColorFn;
drawDebugArrow DrawDebugArrowFn;

void SetDrawDebugBallFn(drawDebugBall callback) { DrawDebugBallFn = callback; }

void SetDrawDebugBallFn(drawDebugBallWithOri callback) { DrawDebugBallWithOriFn = callback; }

void SetDrawDebugBallFn(drawDebugBallWithColor callback) { DrawDebugBallWithColorFn = callback; }

void SetDrawArrowDebugFn(drawDebugArrow callback) { DrawDebugArrowFn = callback; }

void DrawDebugBall(Vec3 pos) { DrawDebugBall(pos); }

void DrawDebugBall(Vec3 pos, Quat q) { DrawDebugBallWithOriFn(pos, q); }

void DrawDebugBall(Vec3 pos, Vec3 color) { DrawDebugBallWithColorFn(pos, color); }

void DrawDebugArrow(Vec3 pos, Vec3 normal) { DrawDebugArrowFn(pos, normal); }
} // namespace ce
