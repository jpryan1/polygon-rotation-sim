#include "animation.h"
#include <thread>
#include <iostream>
#include <vector>
#include "Poly.h"
#include "Collision.h"
#include <ctime>

#define DEFAULT_NUM_POLY_SIDES 8
#define DEFAULT_POLY_RADIUS 8
#define DEFAULT_COEF_OF_RESTITUTION 1
#define DEFAULT_ANIMATION_TIMESTEP 1e-6

#define DEFAULT_SWIRL_NUM_TO_END 13
#define MEASURING_WINDOW_SIZE 1000

void simulation(int, double, double, int, bool);


int main(int argc, char** argv){
	
	std::cout.precision(16);

	int num_poly_sides = DEFAULT_NUM_POLY_SIDES;
	int swirl_num_to_end = DEFAULT_SWIRL_NUM_TO_END;
	double poly_radius = DEFAULT_POLY_RADIUS;
	double coef_of_restitution = DEFAULT_COEF_OF_RESTITUTION;
	bool show_animation = false;
	double animation_timestep = DEFAULT_ANIMATION_TIMESTEP;
	int showForceVec = 0;
	bool show_m_frame = false;

	int c;
	while ((c = getopt (argc, argv, "s:i:r:e:amfd:h")) != -1) {
		switch (c)
		{
		case 's':
			num_poly_sides = std::strtol(optarg, NULL, 0);
			break;
		case 'i':
			swirl_num_to_end = std::strtol(optarg, NULL, 0);
			break;
		case 'r':
			poly_radius = std::strtod(optarg, NULL);
			break;
		case 'e':
			coef_of_restitution = std::strtod(optarg, NULL);
			break;
		case 'a':
			show_animation = true;
			break;
		case 'm':
		  show_m_frame = true;
		  break;
		case 'd':
			animation_timestep = std::strtod(optarg, NULL);
			break;
		case 'f':
			showForceVec = 1;
			break;
		case 'h':
			printf("PolySim\nOptions: sireamfdh ");
			return 0;
		default:
			abort ();
		}
	}
	printf("Simulating with radius %f, coefficient of restitution %f, \
		%d sides, %d num swirls\n", poly_radius, coef_of_restitution, num_poly_sides, swirl_num_to_end);
	

	if(!show_animation){
		Poly::animation = NULL;
		simulation(num_poly_sides, poly_radius, coef_of_restitution, swirl_num_to_end, show_m_frame);
		return 0;
	}
	else{
		Animation animation(animation_timestep, poly_radius, num_poly_sides);
		animation.setup();
		Poly::animation = &animation;

		//Set the simulation running
		std::thread drawer(simulation, num_poly_sides, poly_radius, coef_of_restitution, swirl_num_to_end, show_m_frame);
		//Begin the animation
		animation.draw();
		//Wait for the simulation to finish
		drawer.join();
		return 0;
	}
}


void simulation(int num_poly_sides, double poly_radius, double coef_of_restitution, int swirl_num_to_end, bool show_m_frame){
	Poly poly(num_poly_sides, poly_radius, coef_of_restitution);
	poly.initialize();
	poly.show_m_frame = show_m_frame;
	
	double total_time = 0;
	double total_ang_vel = 0;

  std::vector<double> angvels;

  double window_size = MEASURING_WINDOW_SIZE;
  double window_mean = 0;

  double next_update = BOUNDARY_PERIOD*3;
  
  while(total_time < BOUNDARY_PERIOD * swirl_num_to_end){
    std::vector<Collision> currentCollisions;
		poly.nextCollisions(currentCollisions);
		poly.updatePositions(currentCollisions[0].getTime());
		for(int i=0; i<currentCollisions.size(); i++){
			poly.processCollision(currentCollisions[i]);
		}
		
		total_time += currentCollisions[0].getTime();

    // Every 0.025 seconds...
		if(currentCollisions[0].getType() == SWIRL){
		  // Record the angvel
		  angvels.push_back(poly.getAngVel());
		  if(angvels.size()<window_size){
		    // Keep a running tab on the average
		    window_mean += poly.getAngVel();
		  }else{
        window_mean -= angvels[angvels.size()-1-window_size];
        window_mean += angvels[angvels.size()-1];
        if(std::abs(total_time - next_update) < 1e-6){
          std::cout<<next_update<<","<<window_mean/window_size<<std::endl;
          next_update += 2.5;
        }
		  }
		  poly.updateAnimation(-1);
		}
	}
}
