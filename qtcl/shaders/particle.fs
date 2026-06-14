#version 330 core

in vec4 Color;
in vec2 TexPos;

uniform bool      useTexture;
uniform sampler2D textureId;

void main() {
  if (useTexture) {
    vec4 tc = texture(textureId, TexPos);
    if (tc.a > 0.0)
      gl_FragColor = vec4(Color.rgb, tc.a*Color.a);
    else
      gl_FragColor = vec4(0, 0, 0, 0);
    //gl_FragColor = vec4(tc.rgb, tc.a*Color.a);
    //gl_FragColor = tc;
  } else {
    gl_FragColor = vec4(Color.rgb, 0.5);
  }
}
