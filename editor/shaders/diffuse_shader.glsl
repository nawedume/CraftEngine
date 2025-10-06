#version 330 core

in vec3 vPos;
in vec3 vNormal;
in vec3 vBarycentricCoord;
flat in uvec3 vEdgeMask;

uniform vec3 uBaseColor;

out vec4 fColor;

void main() {
    vec3 bcc = vBarycentricCoord;
    bcc.x = (vEdgeMask.x != 0u) ? bcc.x : 1e6;
    bcc.y = (vEdgeMask.y != 0u) ? bcc.y : 1e6;
    bcc.z = (vEdgeMask.z != 0u) ? bcc.z : 1e6;

    float dist = min(bcc.x, min(bcc.y, bcc.z));

    vec3 color = dist < 0.03 ? vec3(0.2, 0.3, 0.3) : uBaseColor;
    fColor = vec4(color, 1.0);
    //fColor = vec4(vBarycentricCoord, 1.0);
    //fColor = vec4(vEdgeMask, 1.0);
}
