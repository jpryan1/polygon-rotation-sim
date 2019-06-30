#include "Poly.h"

Animation* Poly::animation;


Poly::Poly(int s, double r, double c){
	sides = s;
	radius = r;
	coef = c;

	showForceVec = 0;
 	mass = (0.5) * sides * radius * radius * sin(2*M_PI/sides);
	moment_of_inertia =
		 (0.5) * mass * radius * radius *(1 - (2.0 / 3.0) * pow(sin( M_PI / sides), 2));
}
		
		
void Poly::initialize(){
	for(int i=0; i<NUM_BINS; i++){
	  counts[i] = 0;
	  difs[i] = 0.0;
	}
	
	this->boundvel[0] = AMPLITUDE_PARAMETER;
	this->boundvel[1] = 0;
	this->swirl_time=0;
	this->swirl_counter = TIMESTEPS_PER_SIDE / 2.0;
	
	// Start at halfway through the first side
	this->boundpos[0] =  (TIMESTEPS_PER_SIDE*TIMESTEP_INTERVAL/2.0) * this->boundvel[0];
	this->boundpos[1] =0;
	
	
	
	for(int i=0; i<2; i++){
		centerpos[i] = boundpos[i];
	}
  srand((unsigned)time(NULL));
  double X = ((double) rand() / (double) RAND_MAX);

  double rand_ang = X * 2*M_PI ;
  centervel[0] = cos(rand_ang);
  centervel[1] = sin(rand_ang);
  
	this->ang = 0;
	this->angvel = 0.0;
	
	if(animation){
	  double traj_pos[2];
	  //Doesn't really matter at t=0
	  traj_pos[0] = 0;
	  traj_pos[1] = 0;
	  
		animation->setPoly(centerpos, centervel, ang, angvel, boundpos, boundvel, NULL,0, traj_pos);
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
  // So, the boundary traj starts at 0,0, has side length TIMESTEP_INTERVAL, and
  // has 2pi/SWIRL_ANGLE sides. its center is at an angle SWIRL_ANGLE/2 from origin
  //, distance traj_radius/2 away, traj_radius =
 
  double n = 2 * M_PI/SWIRL_ANGLE;
  double traj_radius = (TIMESTEP_INTERVAL)/(2*sin(M_PI/n));
  double traj_cx = traj_radius*cos(SWIRL_ANGLE/2.0);
  double traj_cy = traj_radius*sin(SWIRL_ANGLE/2.0);

  double rx = centerpos[0] - traj_cx;
  double ry = centerpos[1] - traj_cy;
  
  double period = TIMESTEP_INTERVAL*n;
  double omega = 2*M_PI/period;
  double crossx = -omega*ry;
  double crossy = omega*rx;
  
  return moment_of_inertia * pow(angvel-omega,2);
    +mass*(pow(centervel[0] - crossx, 2)+pow(centervel[1] - crossy,2));
}


double Poly::getEnergyInBFrame(){
  return moment_of_inertia * pow(angvel,2)
    + mass*((centervel[0]-boundvel[0],2)+pow(centervel[1]-boundvel[1],2));
}
