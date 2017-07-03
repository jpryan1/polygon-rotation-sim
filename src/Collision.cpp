#include "Collision.h"


Collision::Collision(const Collision &other){
	type = other.type;
	time = other.time;
	
}


double Collision::getTime(){
	return this->time;
}


Type Collision::getType(){
	return this->type;
}
