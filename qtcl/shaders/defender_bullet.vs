#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;

out vec4 Color;

uniform mat4 view;
uniform mat4 model;

uniform vec3 position;

void main() {
  Color = vec4(aColor, 1);

  gl_Position = view*vec4(position, 1.0);
}
