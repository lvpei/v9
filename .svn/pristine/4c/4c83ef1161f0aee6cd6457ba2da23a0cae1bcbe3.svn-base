attribute vec3 positionIn;
attribute vec2 texcoordIn;
attribute vec3 normalIn;

varying vec2 texcoord;
varying vec3 normal;
varying vec3 eyePosition;
varying vec4 fragPosition;

void main() {

	vec4 eyeTemp = gl_ModelViewMatrix * vec4(positionIn, 1);
	eyePosition = eyeTemp.xyz;

	gl_Position = gl_ProjectionMatrix * eyeTemp;
	
	fragPosition = gl_Position;
	
	normal = gl_NormalMatrix * normalIn;
	texcoord = texcoordIn;

}
