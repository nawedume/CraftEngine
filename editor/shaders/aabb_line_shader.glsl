#version 330 core

in vec3 vPos;
in vec3 iPos;
in vec3 iExtent;

uniform mat4 uWorldTransform;
uniform mat4 uCameraTransform;
uniform mat4 uProjTransform;

void main() {
    vec3 pos = (vPos * iExtent) + iPos;
    gl_Position = uProjTransform * uCameraTransform * vec4(pos, 1.0);
}
