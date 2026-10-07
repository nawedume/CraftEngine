#version 330 core

vec4 fColor;

uniform vec3 uColor;

void main() {
    fColor = vec4(uColor, 1.0);
}
