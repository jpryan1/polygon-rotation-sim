#include "Poly.h"


void Poly::nextCollisions(std::vector<Collision>& currentCollisions){
	currentCollisions.clear();
	checkPolyCollisions( currentCollisions );
	checkSwirlCollision( currentCollisions );
}


void Poly::checkPolyCollisions(std::vector<Collision>& currentCollisions){
	Collision c;
	for(int i=0; i<sides; i++){
		c = nextPolyCollision(i);
		if(c.getTime()==-1) continue;
		c.hit_vertex=i;
		addCollision(currentCollisions, c);
	}
}


Collision Poly::nextPolyCollision(int a){
  
	double first_time = 0; // We perform Newton's method starting at 0, 0.01, 0.1
	int iterations = 0;
	double f_time = newton_f(first_time, a);
	
	while( fabs(f_time) > 1e-13 && iterations++ < 100){
		
		double deriv = newton_fd(first_time, a);
		
		if(fabs(deriv)<1e-12){
			break;
		}
		
		double step_time = first_time;
	  
		first_time = first_time - (f_time/deriv);
		
		f_time = newton_f(first_time, a);
		
	}
	if(fabs( f_time) >1e-8 || first_time<=1e-10){
		first_time = -1;
	}
	
	double second_time = 1e-1;
	iterations = 0;

	f_time = newton_f(second_time, a);
  
	while( fabs(f_time) > 1e-13 && iterations++ < 100){
		
		double deriv = newton_fd(second_time, a);
		
		if(fabs(deriv)<1e-14){
			break;
		}
		
		second_time = second_time - (f_time/deriv);
		
		f_time = newton_f(second_time, a);
		
	}
	if(fabs( f_time) >1e-8 || second_time<=1e-10 ){
		second_time = -1;
  }

  double third_time = 1e-1;
	iterations = 0;

	f_time = newton_f(third_time, a);
  
	while( fabs(f_time) > 1e-13 && iterations++ < 100){
		
		double deriv = newton_fd(third_time, a);
		
		if(fabs(deriv)<1e-14){
			break;
		}
		
		third_time = third_time - (f_time/deriv);
		
		f_time = newton_f(third_time, a);
		
	}
	if(fabs( f_time) >1e-8 || third_time<=1e-10 ){
		third_time = -1;
  }
  
  std::vector<double> possible_times;
  possible_times.push_back(first_time);
  possible_times.push_back(second_time);
  possible_times.push_back(third_time);
  
  double min = 1000000;
  for(double pos_time : possible_times){
    if(pos_time>0){
      if(pos_time<min){
        min = pos_time;
      }
    }
  }
  
  if(min == 1000000){
    return Collision(-1, POLY_WITH_WALL);
  }else{
    return Collision(min, POLY_WITH_WALL);
  }
}


double Poly::newton_f(double t, int a){
  
	double f = 0;
	double A = centerpos[0] - boundpos[0];
	double B = centerpos[1] - boundpos[1];
	double theta = ang + a * (2.0*M_PI / sides) + angvel * t;
	double rc = radius * cos(theta);
	double rs = radius * sin(theta);
	double v0 = centervel[0] - boundvel[0];
	double v1 = centervel[1] - boundvel[1];
	
	f += pow(A,2) + pow(B,2) + pow(radius,2);
	f += 2*t*(v0*A + v1*B);
	f += pow(t,2) * (pow(v0,2) + pow(v1,2));
	f += 2 * (A*rc + B * rs);
	f += 2*t*(v0*rc + v1*rs);
	f -= pow(BOUNDARY_RADIUS,2);

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
	
	Collision swirl_collision(TIMESTEP_INTERVAL - swirl_time, SWIRL);
	bool swirl_added = addCollision(currentCollisions, swirl_collision);
	if(swirl_added){
	  swirl_time = 0;
	}else{
	  swirl_time += currentCollisions[0].getTime();
	}
}


bool Poly::addCollision(std::vector<Collision>& currentCollisions, Collision& collision){
	// if currentcollisions is empty, just add
	// Time of vector is found by first collision. If we're within 1e-13, we add.
	// If we're 1e-13 faster than first one, we delete all that aren't as fast
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
