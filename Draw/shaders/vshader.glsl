#version 330 core

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec3 inNormal;

out vec3 vPos;
out vec3 vNormal;

uniform mat4 uWorldTransform;
uniform mat4 uViewTransform;
uniform mat4 uProjectionTransform;

void main() {
    vec4 worldPos = uWorldTransform * vec4(inPos, 1.0);
    gl_Position = uProjectionTransform * uViewTransform * worldPos;
    vPos = worldPos.xyz;
    vNormal = normalize(inverse(transpose(mat3(uWorldTransform))) * inNormal);

}
