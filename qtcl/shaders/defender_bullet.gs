#version 330 core

layout (points) in;
layout (line_strip, max_vertices = 10) out;

uniform mat4 projection;

void main() {
  float dx  = 2.0;
  float dx1 = 0.5;

  float x1 = 0.0;
  float x2 = 1.0;

  for (int i = 0; i < 5; ++i) {
    x2 = x1 + i*dx + dx1;

    gl_Position = projection*(gl_in[0].gl_Position + vec4(x1, 0, 0, 0));
    EmitVertex();

    gl_Position = projection*(gl_in[0].gl_Position + vec4(x2, 0, 0, 0));
    EmitVertex();

    EndPrimitive();

    x1 = x2 + dx1;
  }
}
