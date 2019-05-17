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


class Collision;

class Poly{
	
	public:
	
		static double swirl_interval;
		static double boundrad;
		static double swirl_angle;
	
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
  	void updateAnimation(double time, int hit_vertex);
		void draw();
  	
  	
	private:
		int sides;

		double coef, radius, ang, angvel, mass, moment_of_inertia;
		double centerpos[2];
		double centervel[2];

	
		double boundpos[2];
		double boundvel[2];
	
		double swirl_time;
};


#endif
