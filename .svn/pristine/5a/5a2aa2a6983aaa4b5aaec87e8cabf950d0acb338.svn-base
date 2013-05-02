uniform sampler2D diffuseMap;
uniform sampler2D reflectionMap;
uniform sampler2D shadowMap;

varying vec2 texcoord;
varying vec3 normal;
varying vec3 eyePosition;
varying vec4 fragPosition;

void main() {

	// Shadow Map
	float blurSizeShadow = (1.0/256.0) * 2.5 / length(eyePosition.xyz);;
	//float blurSizeShadow = 1.0/128.;
	float blurSizeReflection = 1.0/256.0;
	
	float frag_x = 0.5 * (fragPosition.x/fragPosition.w) + 0.5;
	float frag_y = 0.5 * (fragPosition.y/fragPosition.w) + 0.5;
	vec2 vTexCoord = vec2(frag_x,frag_y);
	//float shadowCoeff = texture2D(shadowMap,vec2(frag_x,frag_y)).r;
	
	float sumShadow = 0.;
	float sumShadow2 = 0.;
	
	/*sumShadow += texture2D(shadowMap, vec2(vTexCoord.x - 4.0*blurSizeShadow, vTexCoord.y)).r * 0.05;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x - 3.0*blurSizeShadow, vTexCoord.y)).r * 0.09;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x - 2.0*blurSizeShadow, vTexCoord.y)).r * 0.12;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x - blurSizeShadow, vTexCoord.y)).r * 0.15;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y)).r * 0.16;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x + blurSizeShadow, vTexCoord.y)).r * 0.15;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x + 2.0*blurSizeShadow, vTexCoord.y)).r * 0.12;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x + 3.0*blurSizeShadow, vTexCoord.y)).r * 0.09;
	sumShadow += texture2D(shadowMap, vec2(vTexCoord.x + 4.0*blurSizeShadow, vTexCoord.y)).r * 0.05;
	
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y - 4.0*blurSizeShadow)).r * 0.05;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y - 3.0*blurSizeShadow)).r * 0.09;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y - 2.0*blurSizeShadow)).r * 0.12;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y - blurSizeShadow)).r * 0.15;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y)).r * 0.16;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y + blurSizeShadow)).r * 0.15;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y + 2.0*blurSizeShadow)).r * 0.12;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y + 3.0*blurSizeShadow)).r * 0.09;
	sumShadow2 += texture2D(shadowMap, vec2(vTexCoord.x, vTexCoord.y + 4.0*blurSizeShadow)).r * 0.05;
	*/
	
	float result = 0.;
	
	vec2 texCoord;
	mat3 gaussianCoef = mat3( 1.0, 2.0, 1.0, 2.0, 4.0, 2.0, 1.0, 2.0, 1.0 );
	//vec3 gaussianCoef = vec3(1. , 2. , 1.);
	float stepU = blurSizeShadow;
	float stepV = blurSizeShadow;
	
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<5;j++)
		{
			texCoord = vTexCoord.xy + vec2(float(i-2)*stepU,float(j-2)*stepV);
			vec3 coef = gaussianCoef[i];
			//result += coef[j] * texture2D(shadowMap,texCoord).r;
			result += texture2D(shadowMap,texCoord).r;
		}
	}
	//result /= 16.0;
	result /= 25.;
	
	//float shadowCoeff = 0.5*sumShadow + 0.5*sumShadow2;
	float shadowCoeff = result;
	
	
	float specShadow = 1.;
	if (shadowCoeff > 0.){ specShadow = 0.; }
	
	vec3 N = normalize(normal);
	vec3 L = normalize(gl_LightSource[0].position.xyz);
	//vec3 V = normalize(-eyePosition);
		
	float Rd = max(0.0, dot(L, N));
	vec3 Td = texture2D(diffuseMap, texcoord).rgb;
	vec3 diffuse = Td * Rd * gl_LightSource[0].diffuse.rgb;
	
	vec3 R = reflect(-L, N);
	//float Rs = pow(max(0.0, dot(V, R)), alpha);
	//vec3 specular = Rs * Ks * Ts * gl_LightSource[0].specular.rgb;
		
	vec3 specular = vec3(0.,0.,0.);
	//vec3 ambient = gl_LightSource[0].ambient.rgb;
	float dist = 1.;
	//if (fragPosition.z > 1.) { 
	//dist = 1. - log(1.-(eyePosition.z/2.)/15.); 
    dist = 1. - log(1.-eyePosition.z/30.);
    //}

	vec4 floorColor = vec4(dist * shadowCoeff * (diffuse + specShadow*specular), 1);
	//gl_FragColor = vec4(texture2D(reflectionMap,vec2((0.5*eyePosition.x+0.5),1.-(0.5*eyePosition.y+0.5))).rgb,1.);
	//gl_FragColor = vec4(texture2D(reflectionMap,vec2(gl_FragCoord.x / 840., 1.-gl_FragCoord.y / 442.)).rgb,1.);
	
	float reflected_x = frag_x;
	float reflected_y = 1. - frag_y;
	
	vec4 sum = vec4(0.0);
	vec4 sum2 = vec4(0.0);
	
	vTexCoord = vec2(reflected_x,reflected_y);
	
	/*sum += texture2D(reflectionMap, vec2(vTexCoord.x - 4.0*blurSizeReflection, vTexCoord.y)) * 0.05;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x - 3.0*blurSizeReflection, vTexCoord.y)) * 0.09;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x - 2.0*blurSizeReflection, vTexCoord.y)) * 0.12;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x - blurSizeReflection, vTexCoord.y)) * 0.15;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y)) * 0.16;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x + blurSizeReflection, vTexCoord.y)) * 0.15;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x + 2.0*blurSizeReflection, vTexCoord.y)) * 0.12;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x + 3.0*blurSizeReflection, vTexCoord.y)) * 0.09;
	sum += texture2D(reflectionMap, vec2(vTexCoord.x + 4.0*blurSizeReflection, vTexCoord.y)) * 0.05;
	*/
	
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y - 4.0*blurSizeReflection)) * 0.05;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y - 3.0*blurSizeReflection)) * 0.09;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y - 2.0*blurSizeReflection)) * 0.12;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y - blurSizeReflection)) * 0.15;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y)) * 0.16;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y + blurSizeReflection)) * 0.15;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y + 2.0*blurSizeReflection)) * 0.12;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y + 3.0*blurSizeReflection)) * 0.09;
	sum2 += texture2D(reflectionMap, vec2(vTexCoord.x, vTexCoord.y + 4.0*blurSizeReflection)) * 0.05;
	
	
	//vec4 reflectionColor = 0.5 * sum + 0.5 * sum2;
	vec4 reflectionColor = sum2;
	
	//gl_FragColor = floorColor + 0.5*(1.-reflected_y)*vec4(texture2D(reflectionMap,vec2(reflected_x,reflected_y)).rgb,1.);
	gl_FragColor = floorColor + 0.5*(1.-reflected_y)*reflectionColor;
}
