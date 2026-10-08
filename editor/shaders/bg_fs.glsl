#version 330 core

in vec3 fPos;

out vec4 outColor;

void main() {
    vec3 dir = normalize(fPos);
    float pitch = max(0.0, (asin(dir.y)));

    vec3 deep_blue = vec3(0.0, 0.74, 1.0);
    vec3 sky_purple = vec3(0.698, 0.662, 0.925);

    float alpha = pitch / (3.14259 / 2.0);
    vec3 color = mix(sky_purple, deep_blue, alpha);
    outColor = vec4(color, 1.0);
}
