#ifndef  _POLY_H_    /* only process this file once */
#define  _POLY_H_

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include "vec.h"
#include "type.h"
#include "animation.h"
#include "Collision.h"

#define NUM_BINS 25
#define NUM_SIDES_BOUNDARY_TRAJECTORY 5
#define TIMESTEP_INTERVAL 0.025
#define BOUNDARY_RADIUS 8.6
#define BOUNDARY_PERIOD 12.0
#define AMPLITUDE_PARAMETER 1.0


#define SWIRL_ANGLE (2*M_PI/NUM_SIDES_BOUNDARY_TRAJECTORY)
#define TIMESTEPS_PER_SIDE BOUNDARY_PERIOD/(TIMESTEP_INTERVAL*NUM_SIDES_BOUNDARY_TRAJECTORY)

class Collision;

class Poly{
	
	public:
	
	  static Animation* animation;
	
		Poly(int s, double r, double c);
		~Poly(){}
	
		void initialize();
		void updatePositions(double collisionTime);
	 
		void nextCollisions(std::vector<Collision>& currentCollisions);
		
  	vec vert_pos(int a);
  	double getAngVel();
  	
		Collision nextPolyCollision(int a);

  	double newton_f(double t, int a);
  	double newton_fd(double t, int a);
	
		void checkPolyCollisions( std::vector<Collision>& currentCollisions );
		void checkSwirlCollision( std::vector<Collision>& currentCollisions );
		bool addCollision(std::vector<Collision>& currentCollisions, Collision& collision);

	  double getEnergyInLabFrame();
	  double getEnergyInBFrame();
	  double getEnergyInMFrame();
	  
  	void processCollision(Collision& collision);
  	void processPolyCollision(Collision& collision);
		void swirl();
	
  	double error;
  	
  	int showForceVec;
  	void updateAnimation(int hit_vertex);
		void draw();
  	
  	bool show_m_frame = false;
  	int counts[NUM_BINS];
  	double difs[NUM_BINS];
  	int swirl_counter = 0;
	private:
		int sides;

		double coef, radius, ang, angvel, mass, moment_of_inertia;
		double centerpos[2];
		double centervel[2];

	
		double boundpos[2];
		double boundvel[2];
	
		double swirl_time;
		double total_time = 0.0;
};


#endif
