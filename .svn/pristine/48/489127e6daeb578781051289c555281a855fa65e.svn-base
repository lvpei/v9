varying vec3 normal;
varying vec3 eyePosition;

void main() {

	vec3 N = normalize(normal);
	vec3 L = normalize(gl_LightSource[0].position.xyz);
	vec3 V = normalize(-eyePosition);
	
	vec3 Kd = gl_Color.rgb;
		
	float Rd = max(0.0, dot(L, N));
	vec3 diffuse = Kd * Rd * gl_LightSource[0].diffuse.rgb;
	
	/*vec3 R = reflect(-L, N);
	float Rs = pow(max(0.0, dot(V, R)), alpha);
	vec3 specular = Rs * Ks * Ts * gl_LightSource[0].specular.rgb;
		*/
	vec3 specular = vec3(0.,0.,0.);
	vec3 ambient = gl_LightSource[0].ambient.rgb;
	
	if(Kd.b < 0.5 && Kd.g > 0.98)
		gl_FragColor = vec4(Kd,1.);
	else
		gl_FragColor = vec4(diffuse + specular + ambient, 1);
	
	//gl_FragColor = vec4(1.,0,0,1);
	//gl_FragColor = vec4(eyePosition.z,eyePosition.z,eyePosition.z,1);
}
