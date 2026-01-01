#version 330 core
out vec4 FragColor;
uniform vec3 u_Color; // Podem passar un color per objecte
void main()
{
    FragColor = vec4(u_Color, 1.0);
}