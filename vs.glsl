#version 330 core
layout (location = 0) in vec3 aPos;

uniform mat4 u_Model;
uniform mat4 u_View;
uniform mat4 u_Projection;

void main()
{
    // TODO: Calcular gl_Position
    gl_Position = vec4(0.0, 0.0, 0.0, 1.0);
}