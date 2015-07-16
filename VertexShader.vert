#version 330 core

in vec2 vert_vertex;

uniform vec2 vert_scale;
uniform vec2 vert_offset;

void main() {
	gl_Position = vec4(vert_vertex.x * vert_scale.x + vert_offset.x, vert_vertex.y * vert_scale.y + vert_offset.y, 0.0, 1.0);
}
