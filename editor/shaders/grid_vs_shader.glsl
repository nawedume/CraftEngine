#version 330 core

layout (location = 0) in vec3 inPos;
layout (location = 1) in vec2 inUv;

uniform float uScale;
uniform mat4 uCameraTransform;
uniform mat4 uProjTransform;

out vec2 vUv;

void main() {
    gl_Position = uProjTransform * uCameraTransform * vec4(inPos * uScale, 1.0);
    vUv = inUv;
}
