#include "Poly.h"

double Poly::swirl_interval = 1;
double Poly::boundrad = 8.6;
double Poly::swirl_angle = 3.14159265359 / 6;


Animation* Poly::animation;

void Poly::initialize(){
	
	this->boundpos[0] = 0;
	this->boundpos[1] = 0;
	this->boundvel[0] = 0.5;//0.5;//0.5; //CONSTANT HERE
	this->boundvel[1] = 0;
	this->swirl_time=0;
	for(int i=0; i<2; i++){
		centerpos[i] = 0;
		centervel[i] = 1;
	}
	centerpos[0]+=0.24252312323;
	
	this->ang = 0;
	this->angvel = 0;
	
	
	
	if(animation){
		animation->setPoly(centerpos, centervel, ang, angvel, boundpos, boundvel,NULL);
		animation->notReady = false;
	}
	
	
}
vec Poly::vert_pos(int a){
	return vec(centerpos).add(vec(cos(ang+a*(M_PI/(sides/2.0))),sin(ang+a*(M_PI/(sides/2.0)))).times(radius));
}

double Poly::getAngVel(){
	return angvel;
}









