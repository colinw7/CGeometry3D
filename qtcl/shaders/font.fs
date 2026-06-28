#version 330 core

in vec3 Color;
in vec2 TexCoords;

uniform sampler2D mainTex;

void main() {
  vec4 textureColor = texture(mainTex, TexCoords);

  if (textureColor.r < 0.001)
    discard;

  gl_FragColor = vec4(Color, textureColor.r);
}
