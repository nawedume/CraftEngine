#version 330 core

in vec3 vPos;
in vec3 vNormal;

out vec4 fColor;

uniform vec3 uGlobalLightDir;
uniform vec3 uBaseColor;

void main() {
    const float ambientLightConst = 0.5;
    fColor.xyz = max(0.2f, (ambientLightConst + max(0.0f, dot(vNormal, uGlobalLightDir)))) * uBaseColor;
    fColor.w = 1.0f;
    //fColor.xyz = vec3(1.0, 1.0, 1.0);
}
