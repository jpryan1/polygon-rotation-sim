
#include "animation.h"
#include <thread>
#include <iostream>
#include <vector>
#include "Disks.h"
#include "Collision.h"
#include <ctime>

#define NUM_OF_ITERATIONS 1000000.0

void simulation();


int main(int argc, char** argv){
	
	//By default, we use 55 disks. The max that we can use with our input file is 56
	std::cout.precision(16);
	
	
	if(argc<2){
		std::cout<<"Include arg - a for animate or r for run (without animation)"<<std::endl;
		return 0;
	}
	
	
	/*
	 
	 We accept one of three options. 'r' means just run the simulation on NUM_OF_DISKS disks,
	 and output the average angular velocity.
	 'a' is the same, but with an animation included
	 't' means 'test', and runs the simulation for N from 5 to 56, outputting stats results
	
	 */
	
	
	
	
	if(*(argv[1])=='r'){
		
		Disks::animation =NULL;
		simulation();
		return 0;
		
	}
	
	else if(*(argv[1])=='a'){
		
		Animation animation;
		animation.setup();
		Disks::animation = &animation;

		//Set the simulation running
		std::thread drawer(simulation);
		//Begin the animation
		animation.draw();
		//Wait for the simulation to finish
		drawer.join();
		return 0;
		
	}
	

	else{
		
		std::cout<<"Option must be a for animate or r for run (without animation)"<<std::endl;
		return 0;
		
	}

	
}




void simulation(){
	
	std::vector<Collision> currentCollisions;
	Disks poly;
	poly.initialize();

	
	double total_time=0;
	for(int iterations = 0; iterations<NUM_OF_ITERATIONS; iterations++){
		
		poly.nextCollisions(currentCollisions);
//TODO For the polygon model, a simultaneous collision should happen just about never. Consider ditching
//the code that deals with simultaneous collisions
		//std::cout<<currentCollisions[0].getTime()<<" and type "<<currentCollisions[0].getType()<<" and size "<<currentCollisions.size()<<std::endl;
		poly.updatePositions(currentCollisions[0].getTime());
		total_time +=currentCollisions[0].getTime();
		
		for(int i=0; i<currentCollisions.size(); i++){
			poly.processCollision(currentCollisions[i]);
		}
		poly.updateAnimation(currentCollisions[0].getTime());
		
	}
	std::cout<<"Time: "<<total_time<<std::endl;

	//std::cout<< (total_ang_vel/NUM_OF_ITERATIONS)<<std::endl;

}










