#include "Poly.h"


void Poly::updateAnimation(int hit_vertex){
  total_time += TIMESTEP_INTERVAL;
  double n = 2 * M_PI / SWIRL_ANGLE;
  double bv_norm = sqrt(pow(boundvel[0],2)+pow(boundvel[1],2));
  
  double side_length = bv_norm*TIMESTEPS_PER_SIDE*TIMESTEP_INTERVAL;

  double apothem = (side_length)/(2*tan(M_PI/n));
  double traj_cx = side_length / 2.0;
  double traj_cy = apothem ;

  double traj_pos[2];
  traj_pos[0] = traj_cx;
  traj_pos[1] = traj_cy;
  
  
  double period = n * TIMESTEPS_PER_SIDE * TIMESTEP_INTERVAL;
  double rot_ang = -(total_time/period)*2*M_PI;
  
	if(animation){
		if(showForceVec&&hit_vertex>=0){
			vec v_pos = vert_pos(hit_vertex).minus(vec(boundpos));
			animation->setPoly(centerpos, centervel, ang, angvel, boundpos, boundvel,v_pos.a, rot_ang, traj_pos);
		}else{

		  if(show_m_frame){
		    double newpolypos[2];
  		  double newboundpos[2];
  		  newpolypos[0] = centerpos[0] - traj_cx;
  		  newpolypos[1] = centerpos[1] - traj_cy;
  
  		  double temp   = newpolypos[0];
  		  newpolypos[0] = cos(rot_ang)*newpolypos[0] - sin(rot_ang)*newpolypos[1];
  		  newpolypos[1] = sin(rot_ang)*temp + cos(rot_ang)*newpolypos[1];
  		  
  		  newboundpos[0] = boundpos[0] - traj_cx;
  		  newboundpos[1] = boundpos[1] - traj_cy;
  
  		  temp   = newboundpos[0];
  		  newboundpos[0] = cos(rot_ang)*newboundpos[0] - sin(rot_ang)*newboundpos[1];
  		  newboundpos[1] = sin(rot_ang)*temp + cos(rot_ang)*newboundpos[1];
        double dist = sqrt(pow(newboundpos[0],2)+pow(newboundpos[1],2));
		    // std::cout<<"Dist "<<dist<<"   y is "<<newboundpos[1]<<std::endl;
  		  // std::cout<<"Apothem "<<apothem<<std::endl;
  		  traj_pos[0] = 0;
  		  traj_pos[1] = 0;
  		  animation->setPoly(newpolypos, centervel, ang+rot_ang, angvel, newboundpos, boundvel, NULL, rot_ang, traj_pos);
		  }
		  else{
		    
		    animation->setPoly(centerpos, centervel, ang, angvel, boundpos, boundvel,NULL, rot_ang, traj_pos);
		  }
		}
	double delay = 100000.0*TIMESTEP_INTERVAL;
  usleep(delay);
	}
  
}


void Poly::updatePositions(double time_passed){
	ang += time_passed * angvel;
	for(int i=0; i<2; i++){
		boundpos[i] += time_passed*boundvel[i];
		centerpos[i] += time_passed*centervel[i];
	}
	
	// This forloop just tries to fix numerical errors
	for(int i=0; i<sides; i++){
		vec rad = vert_pos(i).minus(vec(boundpos));
		double d = rad.norm();
		if( d-BOUNDARY_RADIUS>1e-8){
			std::cout<<"Vertex "<<i<<" escaped by "<<d-BOUNDARY_RADIUS<<std::endl;
	    std::cout<<"Boundary: "<<(2.0*M_PI/12.0)<<" Polygon: "<<angvel<<std::endl;

			exit(0);
		}
	}
}


void Poly::processCollision(Collision& collision){
	//Obligation here is just to change the trajectories of the affected Poly
	switch(collision.getType()){
		case POLY_WITH_WALL:
			processPolyCollision(collision);
			break;
		case SWIRL:
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

  if(v_relative.dot(n_hat)>0){
    std::cout<<"Neg v_rel"<<std::endl;
    std::cout<<"Error: impacting vertex approaching from outside polygon."<<std::endl;
    std::cout<<"Here is my guess - you are using CoR<1 and your system lost all its energy"<<std::endl;
    std::cout<<"This is an event driven simulation, which will NOT accept a polygon \"sitting\" on the wall"<<std::endl;
    std::cout<<"Check the animation to see if it is snaking. Here are the angular velocities:"<<std::endl;
    std::cout<<"Boundary: "<<(2.0*M_PI/12.0)<<" Polygon: "<<angvel<<std::endl;
    exit(0);
  }
  
	double triple_cross = pow(r1*n2-r2*n1,2);
	
	double C = coef;
	double J = (-(1+C)*v_relative.dot(n_hat)) /(   (1.0/mass) + (triple_cross/moment_of_inertia)) ;
	vec newcenter = center_vel.minus(n_hat.times(J/mass));
	memcpy(centervel, newcenter.a, 2*sizeof(double));
	
	double angchange = (J/moment_of_inertia)*(r1*n2 - r2*n1);

	angvel -= angchange;
	
	vec vertex_vel2(-angvel*r_poly.a[1], angvel*r_poly.a[0]);
	vertex_vel2 = vertex_vel2.add(vec(centervel));
	vec v_relative2 = vertex_vel2.minus(bound_vel).times(-1);

	double current_err = fabs(v_relative2.dot(n_hat) + C*v_relative.dot(n_hat));
  if(std::fabs(current_err)>1e-10){
    std::cout<<"Error too high in process-event"<<std::endl;
    std::cout<<current_err<<std::endl;
    exit(0);
  }
}


void Poly::swirl(){

  swirl_counter++;
  int thresh = (int) TIMESTEPS_PER_SIDE;

  if(swirl_counter % thresh == 0){
  	double v0 = boundvel[0];
  	boundvel[0] = cos(SWIRL_ANGLE) * boundvel[0] - sin(SWIRL_ANGLE) * boundvel[1];
  	boundvel[1] = sin(SWIRL_ANGLE) * v0 + cos(SWIRL_ANGLE) * boundvel[1];
  }
	
}
