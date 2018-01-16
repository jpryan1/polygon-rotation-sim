#ifndef  _COLLISION_H_    /* only process this file once */
#define  _COLLISION_H_


#include <stdlib.h>
#include <iostream>     
#include <fstream>
#include <cmath>
#include "type.h"

class Collision{
	
	public:
		Collision(){}
		Collision(double t, Type ty) : time(t), type(ty) {}
		Collision(const Collision &other);
		double getTime();
		Type getType();
	
		int hit_vertex;
	private:
		Type type;
		double time;
		
	
};


#endif
