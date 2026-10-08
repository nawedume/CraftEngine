#version 330 core

layout (location = 0) in vec2 inPos;

out vec3 fPos;

// What is the horizontal and vertical size in world units at 1 unit distance
uniform vec2 uScaleAt1Unit;
uniform mat3 uCameraLocalToWorld;

void main() {
    gl_Position = vec4(inPos, 0.0, 1.0);
    vec2 pos_plane = uScaleAt1Unit * inPos;

    // transform the pixel position into the direction-space position
    fPos = uCameraLocalToWorld * vec3(pos_plane, -1.0);
}
