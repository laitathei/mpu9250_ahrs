#version 330 core
layout(location = 0) in vec3 position; // set location attribute value of position variable to 0
layout(location = 1) in vec3 color;     // set location attribute value of color variable to 0
out vec3 vertex_color;                  // set color as output in vec3 format
uniform mat4 orientation;               // orientation matrix
void main()
{
	vertex_color = color;
	gl_Position = orientation * vec4(position, 1.0f);
}
