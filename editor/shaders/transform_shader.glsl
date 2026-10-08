#version 330 core

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;
layout (location = 2) in vec3 inBarycentricCoord;
layout (location = 3) in uvec3 inEdgeMask;

uniform mat4 uWorldTransform;
uniform mat4 uCameraTransform;
uniform mat4 uProjTransform;

out vec3 vPos;
out vec3 vNormal;
out vec3 vBarycentricCoord;

void main() {
    vec4 worldPos = uWorldTransform * vec4(inPos, 1.0);
    gl_Position = uProjTransform * uCameraTransform * worldPos;

    vPos = worldPos.xyz;
    // There shouldn't be any non-uniform scale, so the normal world transform should work
    vNormal = mat3(uWorldTransform) * inNormal;

    vBarycentricCoord = inBarycentricCoord;
}
