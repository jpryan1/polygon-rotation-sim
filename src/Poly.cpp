#include "Poly.h"

double Poly::swirl_interval = 0.025;
double Poly::boundrad = 8.6;
double Poly::swirl_angle = 3.14159265359 / 240;


Animation* Poly::animation;

void Poly::initialize(){
	
	this->boundpos[0] = 0;
	this->boundpos[1] = 0;
	this->boundvel[0] = 1;
	this->boundvel[1] = 0;
	this->swirl_time=0;
	for(int i=0; i<2; i++){
		centerpos[i] = 0;
		centervel[i] = 0.6;
	}
// 	centerpos[0]+=0.00052312323;
	
	this->ang = 0;
	this->angvel = 0.0;
	
	
	
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

double Poly::getEnergyInLabFrame(){
  return moment_of_inertia * pow(angvel,2)
    + mass*(pow(centervel[0],2) + pow(centervel[1],2));
}

double Poly::getEnergyInMFrame(){
  // So, the boundary traj starts at 0,0, has side length swirl_interval, and
  // has 2pi/swirl_angle sides. its center is at an angle swirl_angle/2 from origin
  //, distance traj_radius/2 away, traj_radius =
  double n = 2*M_PI/swirl_angle;
  double traj_radius = (swirl_interval)/(2*sin(M_PI/n));
  double traj_cx = traj_radius*cos(swirl_angle/2.0);
  double traj_cy = traj_radius*sin(swirl_angle/2.0);

  double rx = centerpos[0] - traj_cx;
  double ry = centerpos[1] - traj_cy;
  
  
  double period = swirl_interval*n;
  double omega = 2*M_PI/period;
  double crossx = -omega*ry;
  double crossy = omega*rx;
  
  
  
  return moment_of_inertia *pow(angvel-omega,2);
    +mass*(pow(centervel[0] - crossx, 2)+pow(centervel[1] - crossy,2));
}

double Poly::getEnergyInBFrame(){
  return moment_of_inertia * pow(angvel,2)
    + mass*((centervel[0]-boundvel[0],2)+pow(centervel[1]-boundvel[1],2));
}
