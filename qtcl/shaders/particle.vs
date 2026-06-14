#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;
layout (location = 3) in vec2 aTexCoords;

out vec4 Color;
out vec2 TexPos;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

uniform vec3  position;
uniform float size;
uniform float alpha;
uniform vec2  tpos;
uniform vec2  tsize;

void main() {
  Color = vec4(aColor, alpha);

  vec2 TexPos1 = aPos.xy + 0.5; // 0-1

  TexPos = vec2(tsize.x*TexPos1.x + tpos.x, tsize.y*TexPos1.y + tpos.y);

  vec4 position1 = model*vec4(aPos.x*size, aPos.y*size, 0.0, 1.0);

  gl_Position = projection*view*vec4(position, 1.0) + position1;
}
