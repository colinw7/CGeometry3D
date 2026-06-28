#version 330 core

layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
layout (location = 2) in vec3 aColor;
layout (location = 3) in vec2 aTexCoords;

out vec4 FragPos;
out vec3 Normal;
out vec3 Color;
out vec2 TexCoords;

uniform mat4 meshMatrix;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

#define NUM_CLIPS 4

uniform int  numClipPlanes;
uniform vec4 clipPlane[NUM_CLIPS];

out float gl_ClipDistance[NUM_CLIPS];

void main() {
  vec3 position = aPos;
  vec3 norm     = normalize(aNormal);

  position = vec3(meshMatrix*vec4(position, 1.0));

  FragPos = model*vec4(position, 1.0);
  Normal  = mat3(transpose(inverse(model)))*norm;

  Color     = aColor;
  TexCoords = aTexCoords;

  gl_Position = projection*view*FragPos;

  for (int i = 0; i < NUM_CLIPS; ++i) {
    if (i < numClipPlanes)
      gl_ClipDistance[i] = dot(FragPos, clipPlane[i]); 
    else
      gl_ClipDistance[i] = 0;
  }
}
