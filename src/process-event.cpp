#include "Poly.h"


void Poly::updateAnimation(double time, int hit_vertex){
	if(animation){

		animation->movePoly(time);
		if(showForceVec&&hit_vertex>=0){
			vec v_pos = vert_pos(hit_vertex).minus(vec(boundpos));
			
			animation->setPoly(centerpos, centervel, ang, angvel, boundpos, boundvel,v_pos.a);
		}else{
			animation->setPoly(centerpos, centervel, ang, angvel, boundpos, boundvel,NULL);
		}
	}
}

void Poly::updatePositions(double time){
	ang += time * angvel;
	for(int i=0; i<2; i++){
		boundpos[i] += time*boundvel[i];
		centerpos[i] += time*centervel[i];
	}
	
	//This forloop just tries to fix numerical errors
	for(int i=0; i<sides; i++){
		vec rad = vert_pos(i).minus(vec(boundpos));
		double d = rad.norm();
		if(d>boundrad){

			vec newcenter = vec(centerpos).add(rad.times(-fabs(d-boundrad)/rad.norm()));
			memcpy(centerpos, newcenter.a, 2*sizeof(double));
		}
	}


}




void Poly::processCollision(Collision& collision){
	//Obligation here is just to change the trajectories of the affected Poly
	switch(collision.getType()){
		case POLY_WITH_WALL:
			//std::cout<<"Poly"<<std::endl;
			processPolyCollision(collision);
			break;
		case SWIRL:
			//std::cout<<"SW"<<std::endl;
			swirl();
			break;
	}
	
	
}




void Poly::processPolyCollision(Collision& collision){



	vec vertex_pos = vert_pos(collision.hit_vertex);
	vec rad = vertex_pos.minus(vec(boundpos));
	vec n_hat = rad.times(1.0/rad.norm());
	


	vec center_vel(centervel);
	vec bound_vel(boundvel);

	vec r_poly = vertex_pos.minus(centerpos);
	
	
	vec vertex_vel(-angvel*r_poly.a[1], angvel*r_poly.a[0]);
	vertex_vel = vertex_vel.add(center_vel);
	vec v_relative = vertex_vel.minus(bound_vel).times(-1);
	


	double r1 = r_poly.a[0];
	double r2 = r_poly.a[1];
	double n1 = n_hat.a[0];
	double n2 = n_hat.a[1];

	double triple_cross = pow(r1*n2-r2*n1,2);
	double J = (-(1+coef)*v_relative.dot(n_hat)) /(   (1.0/mass) + (triple_cross/moment_of_inertia)   ) ;
	vec newcenter = center_vel.minus(n_hat.times(J/mass));
	memcpy(centervel, newcenter.a, 2*sizeof(double));
	
	angvel -= (J/moment_of_inertia)*(r1*n2 - r2*n1);


	vec vertex_vel2(-angvel*radius*sin(ang + collision.hit_vertex*(M_PI/(sides/2.0))) , 
		angvel*radius*cos(ang + collision.hit_vertex*(M_PI/(sides/2.0))));
	vertex_vel2 = vertex_vel2.add(vec(centervel));
	vec v_relative2 = vertex_vel2.minus(bound_vel).times(-1);
	double current_err = fabs(v_relative2.dot(n_hat) + coef*v_relative.dot(n_hat));
	error+= current_err;
}


void Poly::swirl(){
	double v0 = boundvel[0];
	boundvel[0] = cos(swirl_angle) * boundvel[0] - sin(swirl_angle) * boundvel[1];
	boundvel[1] = sin(swirl_angle) * v0 + cos(swirl_angle) * boundvel[1];
}














