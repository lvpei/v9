uniform sampler2D depthMap;

varying vec3 lightSpacePosition;

void main() {

	// Shadow Map
	float shadowCoeff = 1.;
	if(lightSpacePosition.x <= 1. && lightSpacePosition.x >= 0. && lightSpacePosition.y <= 1. && lightSpacePosition.y >= 0.)
	{
	float zShadow = texture2D(depthMap, lightSpacePosition.xy).r;
	if( zShadow + 0.00005 < lightSpacePosition.z){ shadowCoeff = 0.6;}
	}

	gl_FragColor = vec4(shadowCoeff,shadowCoeff,shadowCoeff,1.);
}
