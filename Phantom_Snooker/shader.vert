#version 400

in vec3 vert;
in vec2 vertTexCoord;

uniform mat4 PV;
//uniform sampler2D tex;
//uniform vec3 camPos;

out vec2 fragTexCoord;

void main() {
    // Pass the tex coord straight through to the fragment shader
    fragTexCoord = vertTexCoord;
    
    gl_Position = PV * vec4(vert, 1);
	//gl_Position = vec4(vert, 1);
}