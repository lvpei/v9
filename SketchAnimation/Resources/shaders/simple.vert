attribute vec3 positionIn;
attribute vec3 normalIn;

void main() {

	// Transform the vertex to get the eye-space position of the vertex
	vec4 eyeTemp = gl_ModelViewMatrix * vec4(positionIn, 1);

	gl_Position = (gl_ProjectionMatrix * eyeTemp);

}
