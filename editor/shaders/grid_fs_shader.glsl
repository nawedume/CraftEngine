#version 330 core

in vec2 vUv;

uniform float uScale;
uniform vec3 uBaseColor;

out vec4 fColor;

void main() {
    // multiply by 2 due to the grid ranging from -scale to scale
    vec2 uv = vUv * uScale * 2.0;

    vec2 ceil_dist = ceil(uv) - uv;
    vec2 floor_dist = uv - floor(uv);
    vec2 dist_vec = min(ceil_dist, floor_dist);
    float dist = min(dist_vec.x, dist_vec.y);

    vec3 color = dist < 0.005 ? uBaseColor : vec3(0.1, 0.1, 0.1);

    // @todo change to translucent coloring
    fColor = vec4(color, 1.0);
}
