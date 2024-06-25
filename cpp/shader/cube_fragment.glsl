#version 330 core
in vec3 vertex_color;       // set input as vertex_color in vec3 format
out vec4 fragment_color;    // set output as fragment_color in vec4 format
void main()
{
	fragment_color = vec4(vertex_color,1.0f);
}