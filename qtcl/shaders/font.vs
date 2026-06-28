#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
layout (location = 2) in vec2 aTexCoords;

out vec3 FragPos;
out vec3 Color;
out vec2 TexCoords;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

uniform vec3 center;
uniform vec3 cameraUp;
uniform vec3 cameraRight;

uniform bool  billboard;
uniform bool  overlay;
uniform float size;

void main() {
  vec3 position  = aPos;
  vec3 position1 = position;

  if      (overlay) {
    //position1 = center + (size*position.x) + (size*position.y);
    position1 = center + size*position;
  }
  else if (billboard) {
    position1 = center + (size*cameraRight*position.x) + (size*cameraUp*position.y);
  }

  FragPos = vec3(model*vec4(position1, 1.0));

  Color     = aColor;
  TexCoords = aTexCoords;

  gl_Position = projection*view*vec4(FragPos, 1.0);
}
