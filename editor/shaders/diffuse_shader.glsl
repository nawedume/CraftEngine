#version 330 core

in vec3 vPos;
in vec3 vNormal;
in vec3 vBarycentricCoord;

uniform vec3 uBaseColor;
uniform vec3 uLightDir;
uniform float uAmbient;

out vec4 fColor;

void main() {
    float intensity = max(0.0, dot(vNormal, uLightDir));
    intensity = min(intensity + uAmbient, 1.0);
    fColor = vec4(uBaseColor * intensity, 1.0);
}
