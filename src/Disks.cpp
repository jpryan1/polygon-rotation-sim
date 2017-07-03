#include "Disks.h"

double Disks::swirl_interval = 1;
double Disks::boundrad = 9.1;
double Disks::swirl_angle = 3.14159265359 / 6;


Animation* Disks::animation;

void Disks::initialize(){
	
	isBall = true;
	this->boundpos[0] = 0;
	this->boundpos[1] = 0;
	this->boundvel[0] = 0.5;//0.5;//0.5; //CONSTANT HERE
	this->boundvel[1] = 0;
	this->swirl_time=0;
	radius =7.5;
	for(int i=0; i<2; i++){
		centerpos[i] = 0;
		centervel[i] = 1;
	}centerpos[0]+=0.24252312323;
	
	this->ang = 0;
	this->angvel = -0.1;
	
	
	
	if(animation){
		animation->setDisks(centerpos, centervel, ang, angvel, boundpos, boundvel);
		animation->notReady = false;
	}
	
	
}

void Disks::updateAnimation(double time){
	if(animation){
		animation->moveDisks(time);
		animation->setDisks(centerpos, centervel, ang, angvel, boundpos, boundvel);
	}
}
void Disks::updatePositions(double time){
	ang += time * angvel;
	for(int i=0; i<2; i++){
		boundpos[i] += time*boundvel[i];
		centerpos[i] += time*centervel[i];
	}
	
	//This forloop just tries to fix numerical errors
	for(int i=0; i<SIDES; i++){
		vec rad = vert_pos(i).minus(vec(boundpos));
		double d = rad.norm();
		if(d>boundrad){
			vec newcenter = vec(centerpos).add(rad.times(-fabs(d-boundrad)/rad.norm()));
			memcpy(centerpos, newcenter.a, 2*sizeof(double));
		}
	}
//	if(animation){
//		animation->moveDisks(time);
//		animation->setDisks(centerpos, centervel, ang, angvel, boundpos, boundvel);
//	}
}
vec Disks::vert_pos(int a){
	return vec(centerpos).add(vec(cos(ang+a*(M_PI/(SIDES/2.0))),sin(ang+a*(M_PI/(SIDES/2.0)))).times(radius));
}

void Disks::nextCollisions(std::vector<Collision>& currentCollisions){
	currentCollisions.clear();
	if(isBall){
		checkBwWCollisions(currentCollisions);
	}
	else{
		checkPwWCollisions( currentCollisions );
	}
	checkSwirlCollision( currentCollisions );
	
}

void Disks::checkPwWCollisions(std::vector<Collision>& currentCollisions){
	Collision c;
	bool flag = false;
	for(int i=0; i<SIDES; i++){
		c = nextPwWCollision(i);
		//std::cout<<"Next collision for "<<i<<" is "<<c.getTime()<<std::endl;
		if(c.getTime()==-1) continue;
		if(addCollision(currentCollisions, c)){
			hit_vertex = i;
			flag = true;
		}
	}
	if(!flag){
	//	std::cout<<"Problem"<<std::endl;
		
	}//std::cout<<"Collision is at "<<hit_vertex<<std::endl;
	
}

double Disks::newton_f(double t, int a){
	double f = 0;
	double A = centerpos[0] - boundpos[0];
	double B = centerpos[1] - boundpos[1];
	double theta = ang + a*(M_PI/(SIDES/2.0)) + angvel*t;
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
double Disks::newton_fd(double t, int a){
	double f = 0;
	double A = centerpos[0] - boundpos[0];
	double B = centerpos[1] - boundpos[1];
	double theta = ang + a*(M_PI/(SIDES/2.0)) + angvel*t;
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

Collision Disks::nextPwWCollision(int a){
	
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

void Disks::checkBwWCollisions(std::vector<Collision>& currentCollisions){
	Collision c = nextBwWCollision();
	if(c.getTime()==-1) return;
	addCollision(currentCollisions, c);
}



Collision Disks::nextBwWCollision(){
	double time;
	double dv0 = centervel[0]-boundvel[0];
	double dv1 = centervel[1]-boundvel[1];
	double dp0 = centerpos[0]-boundpos[0];
	double dp1 = centerpos[1]-boundpos[1];
	
	
	double A = pow(dv0,2) + pow(dv1,2);
	double B = 2 * ( dp0* dv0 + dp1 *dv1);
	double C = pow(dp0,2) + pow(dp1,2) - pow(boundrad-radius, 2);
	double det = B*B-4*A*C;
	
	if(det<0){
		time = -1;
	}else{
		double t = (0.0 - B + sqrt(det))/(2*A);
		if(t>1e-13){
			time = t;
		}
		else{
			time = -1;
		}
	}
	
	return Collision(time, BALL_WITH_WALL);
	
}



void Disks::checkSwirlCollision( std::vector<Collision>& currentCollisions ){
	
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

bool Disks::addCollision(std::vector<Collision>& currentCollisions, Collision& collision){
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



void Disks::processCollision(Collision& collision){
	//Obligation here is just to change the trajectories of the affected disks
	switch(collision.getType()){
		case BALL_WITH_WALL:
			//std::cout<<"BWW"<<std::endl;
			processBwWCollision(collision);
			break;
		case POLY_WITH_WALL:
			//std::cout<<"PWW"<<std::endl;
			processPwWCollision(collision);
			break;
		case SWIRL:
			//std::cout<<"SW"<<std::endl;
			swirl();
			break;
	}
	
	
}


void Disks::processBwWCollision(Collision& collision){
	isBall = false;
	
}

void Disks::processPwWCollision(Collision& collision){
	//isBall = true;
	vec vertex_pos(centerpos[0] + radius*cos(ang+hit_vertex*(M_PI/(SIDES/2.0))), centerpos[1] + radius*sin(ang+hit_vertex*(M_PI/(SIDES/2.0))));
	vec rad = vertex_pos.minus(vec(boundpos));
	vec rad_n = rad.times(1.0/rad.norm());
	
	//First, find velocity vector of striking vertex
	vec center_vel(centervel);
	vec vertex_vel(-angvel*radius*sin(ang + hit_vertex*(M_PI/(SIDES/2.0))) , angvel*radius*cos(ang + hit_vertex*(M_PI/(SIDES/2.0))));
	vertex_vel = vertex_vel.add(center_vel);
	
	//Then, convert to stationary boundary FoR
	vec bound_vel(boundvel);
	vec vertex_in_FoR = vertex_vel.minus(bound_vel);
		//Put vertex in components, flip perp, reduce by 10%
	vec normal_to_boundary = rad_n.times(vertex_in_FoR.dot(rad_n));
	vec tang = vertex_in_FoR.minus(normal_to_boundary);
	vec reaction = normal_to_boundary.times(-0.9);
	vec new_linear = tang.add(reaction);
	new_linear = new_linear.add(bound_vel);
	double force = normal_to_boundary.norm()*1.9*2.59807621135; //This number is the mass, preprocess later
	//F=ma and a = delta_v
	vec force_vec = reaction.times(1.0/reaction.norm()).times(force);
	
	//Apply completely to linear motion,
	memcpy(centervel, new_linear.a, 2*sizeof(double));
	//then take tangential part wrt poly and apply to angular motion
	vec poly_rad = vertex_pos.minus(vec(centerpos));
	double sign = poly_rad.cross(rad);
	double torque = poly_rad.cross(force_vec);
	total_torque += torque;
	std::cout<<total_torque<<std::endl;
	angvel = angvel + (torque/1.0825296);
	angvel = angvel / 50.0;
	//std::cout<<force<<std::endl;
	//sleep(3);
	
}


void Disks::swirl(){
	double v0 = boundvel[0];
	boundvel[0] = cos(swirl_angle) * boundvel[0] - sin(swirl_angle) * boundvel[1];
	boundvel[1] = sin(swirl_angle) * v0 + cos(swirl_angle) * boundvel[1];
}














