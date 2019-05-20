#include "animation.h"
#include <thread>
#include <iostream>
#include <vector>
#include "Poly.h"
#include "Collision.h"
#include <ctime>

#define DEFAULT_NUM_OF_ITERATIONS 2000000.0
#define DEFAULT_SIDES 8
#define DEFAULT_RADIUS 8
#define DEFAULT_E 1
#define DEFAULT_DELTA_T 1e-6

void simulation(int, double, double, int, bool);


int main(int argc, char** argv){
	
	std::cout.precision(16);

	int sides = DEFAULT_SIDES;
	int iterations = DEFAULT_NUM_OF_ITERATIONS;
	double radius = DEFAULT_RADIUS;
	double coef = DEFAULT_E;
	int animate = 0;
	double delta_t = DEFAULT_DELTA_T;
	int c;
	int showForceVec = 0;
	bool m_frame = false;
	
	while ((c = getopt (argc, argv, "s:i:r:e:amfd:h")) != -1) {
		switch (c)
		{
		case 's':
			sides = std::strtol(optarg, NULL, 0);
			break;
		case 'i':
			iterations = std::strtol(optarg, NULL, 0);
			break;
		case 'r':
			radius = std::strtod(optarg, NULL);
			break;
		case 'e':
			coef = std::strtod(optarg, NULL);
			break;
		case 'a':
			animate = 1;
			break;
		case 'm':
		  m_frame = true;
		  break;
		case 'd':
			delta_t = std::strtod(optarg, NULL);
			break;
		case 'f':
			showForceVec = 1;
			break;
		case 'h':
			printf("PolySim\nOptions: erisa ");
			return 0;
		default:
			abort ();
		}
	}

// 	printf("Simulating with radius %f, coefficient of restitution %f, \
		%d sides, %d iterations\n", radius, coef, sides, iterations);
	

	if(!animate){
		Poly::animation =NULL;
		simulation(sides, radius, coef, iterations, false);
		return 0;
	}
	else{
		Animation animation(delta_t, radius, sides);
		animation.setup();
		Poly::animation = &animation;

		//Set the simulation running
		std::thread drawer(simulation, sides, radius, coef, iterations, m_frame);
		//Begin the animation
		animation.draw();
		//Wait for the simulation to finish
		drawer.join();
		return 0;
	}
}


void simulation(int sides, double radius, double coef, int max_iterations, bool m_frame){
	
	std::vector<Collision> currentCollisions;
	Poly poly(sides, radius, coef);
	poly.initialize();
	poly.m_frame = m_frame;
	
	double total_time=0;
	double total_ang_vel=0;

  std::vector<double> angvels;
  
  for(int iterations = 0; iterations<max_iterations; iterations++){
		  
    
		poly.nextCollisions(currentCollisions);
		poly.updatePositions(currentCollisions[0].getTime());
		total_time +=currentCollisions[0].getTime();

		for(int i=0; i<currentCollisions.size(); i++){
			poly.processCollision(currentCollisions[i]);
		}
		poly.updateAnimation(currentCollisions[0].getTime(), -1);
    angvels.push_back(poly.getAngVel());
	}
	
	double mean = 0;
  for(int i=0; i<100000; i++){
	  mean += angvels[i];
	}
	std::cout<<mean/100000.0<<std::endl;
	for(int i=100000; i < angvels.size(); i++){
	  mean -= angvels[i-100000];
	  mean += angvels[i];
	  if(i%100==0){
	    
	  
	  std::cout<<mean/100000.0<<std::endl;
	  }
	    
	 }
}
