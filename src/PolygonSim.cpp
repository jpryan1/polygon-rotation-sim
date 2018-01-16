
#include "animation.h"
#include <thread>
#include <iostream>
#include <vector>
#include "Poly.h"
#include "Collision.h"
#include <ctime>

#define DEFAULT_NUM_OF_ITERATIONS 400000.0
#define DEFAULT_SIDES 8
#define DEFAULT_RADIUS 7.5
#define DEFAULT_E 0.8
#define DEFAULT_DELTA_T 1e-7
void simulation(int, double, double, int, int);
void processHitVertices(std::vector<int>&, int);


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
	while ((c = getopt (argc, argv, "s:i:r:e:afd:h")) != -1) {
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
		case 'd':
			delta_t = std::strtod(optarg, NULL);
			break;
		case 'f':
			showForceVec = 1;
			break;
		case 'h':
			printf("IE-Solver 0.1a\nOptions: erisa ");
			return 0;
		default:
			abort ();
		}
	}

	printf("Simulating with radius %f, coefficient of restitution %f, \
		%d sides, %d iterations\n", radius, coef, sides, iterations);
	/*
	 
	 We accept one of two options. 'r' means just run the simulation on NUM_OF_Poly Poly,
	 and output the average angular velocity.
	 'a' is the same, but with an animation included
	 
	 */
	
	
	
	
	if(!animate){
		
		Poly::animation =NULL;
		simulation(sides, radius, coef, iterations, showForceVec);
		return 0;
		
	}
	
	else{
		
		Animation animation(delta_t, radius, sides);
		animation.setup();
		Poly::animation = &animation;

		//Set the simulation running
		std::thread drawer(simulation, sides, radius, coef, iterations, showForceVec);
		//Begin the animation
		animation.draw();
		//Wait for the simulation to finish
		drawer.join();
		return 0;
		
	}
}




void simulation(int sides, double radius, double coef, int max_iterations, int showForceVec){
	
	std::vector<Collision> currentCollisions;
	Poly poly(sides, radius, coef);
	poly.initialize();
	poly.showForceVec = showForceVec;
	
	double total_time=0;
	double total_ang_vel=0;

	std::vector<int> vert_dists(100000);

	int last_hit_vert = 0;
	int vert_dists_size=0;
	for(int iterations = 0; iterations<max_iterations; iterations++){
		
		poly.nextCollisions(currentCollisions);
		poly.updatePositions(currentCollisions[0].getTime());
		total_time +=currentCollisions[0].getTime();
		if(iterations>300000){
			total_ang_vel += poly.getAngVel();
				
			if(currentCollisions[0].getType() != SWIRL){
				int current_hit_vert = currentCollisions[0].hit_vertex;
				int lower = std::min(last_hit_vert, current_hit_vert);
				int higher = std::max(last_hit_vert, current_hit_vert);
				int dist = std::min(higher-lower, lower-higher+sides);
				vert_dists[vert_dists_size] = dist;
				vert_dists_size++;
				last_hit_vert = current_hit_vert;
			}
		}


		for(int i=0; i<currentCollisions.size(); i++){
			poly.processCollision(currentCollisions[i]);
		}
		if(currentCollisions[0].getType() == SWIRL){
			poly.updateAnimation(currentCollisions[0].getTime(), -1);
		}
		else{
			poly.updateAnimation(currentCollisions[0].getTime(), currentCollisions[0].hit_vertex);
		
		}
	}
	//processHitVertices(vert_dists, vert_dists_size);
	std::cout<< "AngVel: "<<(total_ang_vel/(max_iterations-300000))<<std::endl;
	//printf("Error %f\n", poly.error);
	
}
void processHitVertices(std::vector<int>& vert_dists, int size){
	double total = 0;
	for(int i=0; i<size; i++){
		total += vert_dists[i];
	}
	double mean = total/size;
	std::cout<<mean<<std::endl;

	double dev = 0;
	for(int i=0; i<size; i++){
		dev += pow(vert_dists[i]-mean,2);
	}
	dev /= size;
	dev = sqrt(dev);
	std::cout<<dev<<std::endl;


}









