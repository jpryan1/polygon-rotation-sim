#ifndef  _DISKS_H_    /* only process this file once */
#define  _DISKS_H_


#define NUM_VERTS 6
#define SIDES 15
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


class Disks{
	
	public:
	
		static double swirl_interval;
		static double boundrad;
		static double swirl_angle;
	
	static Animation* animation;
	
		Disks(){
			total_torque = 0;
		}
		~Disks(){
			
		}
	
		void initialize();
		void updatePositions(double collisionTime);
	void updateAnimation(double time);

		void draw();
		void nextCollisions(std::vector<Collision>& currentCollisions);
	
	vec vert_pos(int a);

	
		Collision nextBwWCollision();
		Collision nextPwWCollision(int a);
	
	double newton_f(double t, int a);
	double newton_fd(double t, int a);
	
		void checkBwWCollisions( std::vector<Collision>& currentCollisions );
		void checkPwWCollisions( std::vector<Collision>& currentCollisions );
		void checkSwirlCollision( std::vector<Collision>& currentCollisions );

		bool addCollision(std::vector<Collision>& currentCollisions, Collision& collision);

	
	void processCollision(Collision& collision);
	void processPwWCollision(Collision& collision);
	void processBwWCollision(Collision& collision);
		void swirl();
	
	
	private:
	double total_torque;
		bool isBall;
	int hit_vertex;
		double radius;
		double ang;
		double angvel;
		double centerpos[2];
		double centervel[2];

	
		double boundpos[2];
		double boundvel[2];
	
		double swirl_time;
};


#endif
