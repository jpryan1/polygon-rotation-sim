
#include "stats.h"
#define PI 3.1415926535897932

void Stats::initialize(int N){
	disks = new Disk[N];
	num_of_disks = N;
	M_ang = PI;
	for(int i=0; i<100; i++){
		for(int j=0; j<100; j++){
			hist[i][j] = 0;
		}
		
	}
	for(int i=0;i<30; i++){
		ang_vels[i] = 0;
		ang_counts[i] = 0;
	}
	CoM[0] = 0;
	CoM[1] = 0;
}

void Stats::update(Disk* d, double* b, double time){
	memcpy(disks, d, num_of_disks*sizeof(Disk));
	memcpy(boundpos, b, 2*sizeof(double));
	if(time<0) return;
	
	M_ang -= time*(PI / 6);
	if(M_ang<0) M_ang += (2*PI);
	
	double totalx=0;
	double totaly=0;
	
	for(int i=0; i<num_of_disks; i++){
		disks[i].pos[0] -= boundpos[0];
		disks[i].pos[1] -= boundpos[1];
		
		double temp = disks[i].pos[0];
		disks[i].pos[0] = cos(M_ang)*disks[i].pos[0] - sin(M_ang)*disks[i].pos[1];
		disks[i].pos[1] = sin(M_ang)*temp + cos(M_ang)*disks[i].pos[1];
		
		totalx+=disks[i].pos[0];
		totaly+=disks[i].pos[1];
		
		
		int m = floor(((disks[i].pos[0]+9.1)/18.2)*100.0);
		int n = floor(((disks[i].pos[1]+9.1)/18.2)*100.0);
		hist[n][m] += 1;
		
		temp = disks[i].vel[0];
		disks[i].vel[0] = cos(M_ang)*disks[i].vel[0] - sin(M_ang)*disks[i].vel[1];
		disks[i].vel[1] = sin(M_ang)*temp + cos(M_ang)*disks[i].vel[1];
	}
	double currentCoMx = totalx/num_of_disks;
	double currentCoMy = totaly/num_of_disks;
	
	CoM[0] += currentCoMx;
	CoM[1] += currentCoMy;
	
	for(int i=0; i<num_of_disks; i++){
		double r1 = disks[i].pos[0] - currentCoMx;
		double r2 = disks[i].pos[1] - currentCoMy;
		
		double ang = PI + atan2(r2, r1);
		double rsquared = pow(r1,2) + pow(r2,2);
		double rcrossv = r1*disks[i].vel[1] - r2*disks[i].vel[0];
		int ind = floor((ang/(2*PI))*100);
		ang_vels[ind] += rcrossv/rsquared;
		ang_counts[ind] += 1;
		
	}
}

void Stats::printRad(){
	CoM[0] = CoM[0] / 1000000.0;
	CoM[1] = CoM[1] / 1000000.0;
	double n = sqrt(pow(CoM[0],2) + pow(CoM[1],2));
	std::cout<<9.1-n<<std::endl;
}

void Stats::print(){
//	for(int i=0; i<100; i++){
//		for(int j=0; j<100; j++){
//			std::cout<<hist[i][j]<<std::endl;
//		}
//	}
	double tot = 0;
	for(int i=0; i<100; i++){
		if(ang_counts[i]==0){
			std::cout<<0<<std::endl;
		}
		else{
			std::cout<<ang_vels[i]/ang_counts[i]<<std::endl;
		}
	}std::cout<<tot<<std::endl;
}






