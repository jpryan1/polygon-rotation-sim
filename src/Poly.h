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
	
		Poly(int s, double r, double c){
			sides = s;
			radius = r;
			coef = c;
			total_torque = 0;
			error = 0;
			showForceVec = 0;
		 	mass = (0.5) * sides * radius * radius * sin(2*M_PI/sides);
			moment_of_inertia =
				 (0.5)*mass*radius*radius*(1-(2.0/3.0)*pow(sin(M_PI/sides),2));

		}
		~Poly(){
			
		}
	
		void initialize();
		void updatePositions(double collisionTime);
	  void updateAnimation(double time, int hit_vertex);
		void draw();
		void nextCollisions(std::vector<Collision>& currentCollisions);
		
  	vec vert_pos(int a);
  	double getAngVel();
  	
		Collision nextPolyCollision(int a);
		double experimental_getTimeOfCollision(int a);

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
  	void processBwWCollision(Collision& collision);
		void swirl();
	
  	double error;
  	int showForceVec;
  	
	private:
		int sides;

		double coef, total_torque, radius, ang, angvel, mass, moment_of_inertia;
		double centerpos[2];
		double centervel[2];

	
		double boundpos[2];
		double boundvel[2];
	
		double swirl_time;
};


#endif
