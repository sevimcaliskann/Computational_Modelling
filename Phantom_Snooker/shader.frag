#version 400

uniform sampler2D tex;
uniform vec3 camPos;

in vec2 fragTexCoord;

out vec4 finalColor;

void main() {
	vec3 lPos1 = vec3(-10.0, -10.0, -10.0); 
	vec3 lPos2 = vec3(10.0, 10.0, 10.0); 

	vec3 vPos1 = vec3(0.0, 10.0, 10.0); 
	vec3 vPos2 = vec3(5.0, -5.0, 10.0);

	vec3 n = vec3(0.0, 0.0, 1.0);

	vec3 Id = vec3(1.0, 0.0, 0.0);   
	vec3 Ia = vec3(1.0, 0.0, 0.0);  
	vec3 Is = vec3(1.0, 1.0, 1.0);
    
	 vec3 Ka=vec3  (0.2, 0.2, 0.2);   
	 vec3 Kd=vec3  (0.6, 0.6, 0.6);     
	vec3 Ks =vec3 (1.0, 1.0, 1.0); 

	float s=20;












 

	vec3 v1 = vec3(camPos.x-vPos1.x,camPos.y-vPos1.y,camPos.z-vPos1.z);

	vec3 v=normalize(v1);


	vec3 l1= vec3(lPos1.x-vPos1.x,lPos1.y-vPos1.y,lPos1.z-vPos1.z);  

	vec3 nl1=normalize(l1); 


 

	vec3 l2= vec3(lPos2.x-vPos1.x,lPos2.y-vPos1.y,lPos2.z-vPos1.z);  

	vec3 nl2=normalize(l2); 


	vec3 R1=reflect(-nl1,n);

	vec3 r1=normalize(R1);


	vec3 R2=reflect(-nl2,n);

	vec3 r2=normalize(R2);


 

	float cosAngIncidence1 = max(dot(n, nl1),0);
	float cosAngIncidence2 = max(dot(n, nl2),0);




	float phongTerm1 = max(dot(r1, v),0);
	float phongTerm2 = max(dot(r2, v),0);
	

	vec4 texColor = texture(tex, fragTexCoord);
	vec3 color = vec3(texColor.r, texColor.g, texColor.b);
	vec3 IA=vec3(color.x*Ka.x, color.y*Ka.y, color.z*Ka.z);

	vec3 ID1= vec3(cosAngIncidence1 *Id.x*Kd.x, cosAngIncidence1 *Id.y*Kd.y,cosAngIncidence1 *Id.z*Kd.z);

	vec3 ID2= vec3(cosAngIncidence2 *Id.x*Kd.x, cosAngIncidence2 *Id.y*Kd.y,cosAngIncidence2 *Id.z*Kd.z);



	vec3 IS1= vec3((pow (phongTerm1 ,s))*Is.x*Ks.x,(pow (phongTerm1 ,s))*Is.y*Ks.y,(pow (phongTerm1 ,s))*Is.z*Ks.z);

	vec3 IS2= vec3((pow (phongTerm2 ,s))*Is.x*Ks.x,(pow (phongTerm2 ,s))*Is.y*Ks.y,(pow (phongTerm2 ,s))*Is.z*Ks.z);

	vec4 I;
	I= vec4((IA+ID1+IS1+ID2+IS2),1.0);
	//vec4 I = vec4(color, 1.0);
    finalColor = I;
	//vec2 pos = vec2(camPos.x-fragTexCoord.x,camPos.y-fragTexCoord.y);
	//finalColor = texture(tex, pos);
	//finalColor = texture(tex, fragTexCoord);
}