uniform mat4 lightMv;
uniform mat4 lightProj;
//uniform mat4 modelMatrix;

attribute vec3 positionIn;
attribute vec2 texcoordIn;
attribute vec3 normalIn;

varying vec3 lightSpacePosition;

void main() {

	vec4 eyeTemp = gl_ModelViewMatrix * vec4(positionIn, 1);

	gl_Position = gl_ProjectionMatrix * eyeTemp;
	
	mat4 bias = mat4(0.5,0.,0.,0.,0.,0.5,0.,0.,0.,0.,0.5,0.,0.5,0.5,0.5,1.);

	vec4 lSPTemp = bias* (lightProj * ( lightMv * vec4(positionIn, 1)));

	lightSpacePosition = lSPTemp.xyz;

}
