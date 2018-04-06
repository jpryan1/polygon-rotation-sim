#include "Poly.h"


void Poly::nextCollisions(std::vector<Collision>& currentCollisions){
	currentCollisions.clear();

	checkPolyCollisions( currentCollisions );
	
	checkSwirlCollision( currentCollisions );
	
}

void Poly::checkPolyCollisions(std::vector<Collision>& currentCollisions){
	Collision c;
	bool flag = false;
	for(int i=0; i<sides; i++){
		c = nextPolyCollision(i);
		c.hit_vertex=i;
		//std::cout<<"Next collision for "<<i<<" is "<<c.getTime()<<std::endl;
		if(c.getTime()==-1) continue;
		if(addCollision(currentCollisions, c)){
			flag = true;
		}
	}
	if(!flag){
	//	std::cout<<"Problem"<<std::endl;
		
	}
}


Collision Poly::nextPolyCollision(int a){
	
	double time = 0; //Naive approach for now - start with guess at zero
	int iterations = 0;
	double f_time = newton_f(time, a);
	while( fabs(f_time) > 1e-16 && iterations++ < 100){
		
		double deriv = newton_fd(time, a);
		
		if(fabs(deriv)<1e-8){
			time = -1;
			break;
		}
		
		time = time - (f_time/deriv);
		
		f_time = newton_f(time, a);
		
	}
	if(fabs( f_time) >1e-12 || time<=1e-12 || time>1-1e-12){
		//std::cout<<time<<" -> -1"<<std::endl;
		time = -1;
	}
	
	
	return Collision(time, POLY_WITH_WALL);
}

double Poly::newton_f(double t, int a){
	double f = 0;
	double A = centerpos[0] - boundpos[0];
	double B = centerpos[1] - boundpos[1];
	double theta = ang + a*(M_PI/(sides/2.0)) + angvel*t;
	double rc = radius*cos(theta);
	double rs = radius*sin(theta);
	double v0 = centervel[0] - boundvel[0];
	double v1 = centervel[1] - boundvel[1];
	
	f += pow(A,2) + pow(B,2) + pow(radius,2);
	f += 2*t*(v0*A + v1*B);
	f += pow(t,2) * (pow(v0,2) + pow(v1,2));
	f += 2 * (A*rc + B * rs);
	f += 2*t*(v0*rc + v1*rs);
	f -= pow(boundrad,2);

	return f;
}
double Poly::newton_fd(double t, int a){
	double f = 0;
	double A = centerpos[0] - boundpos[0];
	double B = centerpos[1] - boundpos[1];
	double theta = ang + a*(M_PI/(sides/2.0)) + angvel*t;
	double rc = radius*cos(theta);
	double rs = radius*sin(theta);
	double v0 = centervel[0] - boundvel[0];
	double v1 = centervel[1] - boundvel[1];
	
	f += 2*(v0*A + v1*B);
	f += 2*t*(pow(v0,2) + pow(v1, 2));
	f += 2*angvel*(-A*rs + B*rc);
	f += 2*(v0*rc + v1*rs);
	f += 2*angvel * t * (-v0*rs + v1*rc);
	return f;
}



 


void Poly::checkSwirlCollision( std::vector<Collision>& currentCollisions ){
	
	Collision collision(swirl_interval - swirl_time, SWIRL);
	if(currentCollisions.size()==0){
		swirl_time = 0;
	}
	else if(swirl_interval - swirl_time - currentCollisions[0].getTime() < 1e-13){
		//this will evaluate to true only if the swirl will be added
		swirl_time = 0;
	}else{
		swirl_time += currentCollisions[0].getTime();
	}
	addCollision( currentCollisions, collision);
}

bool Poly::addCollision(std::vector<Collision>& currentCollisions, Collision& collision){
	//if currentcollisions is empty, just add
	//Time of vector is found by first collision. If we're within 1e13, we add.
	//If we're 1e13 faster than first one, we delete all that aren't as fast
	if(currentCollisions.size()==0){
		currentCollisions.push_back(collision);
		return true;
	}
	
	double diff = currentCollisions[0].getTime() - collision.getTime();
	
	if( diff > 1e-13){
		currentCollisions.clear();
		currentCollisions.push_back(collision);
		return true;
	}
	else if(fabs(diff) < 1e-13){
		currentCollisions.push_back(collision);
		return true;
	}
	return false;
}


