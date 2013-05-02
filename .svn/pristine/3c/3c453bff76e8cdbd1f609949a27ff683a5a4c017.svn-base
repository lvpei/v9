attribute vec3 positionIn;
attribute vec3 normalIn;
varying vec2 vTexCoord;


void main(void)
{
   gl_Position = ftransform();;
  	vec4 eyeTemp = gl_ModelViewMatrix * vec4(positionIn, 1);
	vec4 posTemp = gl_ProjectionMatrix * eyeTemp;
   // Clean up inaccuracies
   vec2 Pos;
   Pos = sign(gl_Vertex.xy);
   gl_Position = vec4(Pos, posTemp.z/posTemp.w , 1.0);
	//gl_Position = vec4(Pos,0., 1.0);
   // Image-space
   vTexCoord = Pos * 0.5 + 0.5;
}
