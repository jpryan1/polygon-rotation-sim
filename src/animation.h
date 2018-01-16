
#ifndef  _ANIMATION_H_    /* only process this file once */
#define  _ANIMATION_H_

#define GLEW_STATIC
#include <iostream>
#include "circle.h"
#include "polygon.h"
#include "forcevec.h"
#include <mutex>
#include <atomic>
#include <unistd.h>

class Animation{
	public:
	
		Animation(){}
		Animation( double d, double r, int s){
			notReady = true;
			drawing = false;
			drawForceVec = false;
			boundpos[0] = 0;
			boundpos[1] = 0;
			delta_t = d;
			radius = r;
			sides = s;
		}
		void initialize();
		void setup();
		void compileShaders();
		void generateBuffers();
		void generateShapes();
		void setProjectionMatrices();
		void draw();
		void drawShapes();
		void setPoly(double* pp, double* pv, double pa, double pav,
					 double* b, double* v, double* v_pos);
		void movePoly(double time);
		std::atomic<bool> notReady;
	std::atomic<bool> drawing;
	std::atomic<bool> drawForceVec;

	
	private:
	
		GLuint s_VBO, s_VAO, s_EBO, shaderProgram, modelLoc, colorLoc, viewLoc;
		GLuint b_VBO, b_VAO, b_EBO;
		GLuint f_VBO, f_VAO, f_EBO;
		int width, height;
		GLFWwindow* window;
		Polygon polygon;
		Circle circle;
		Forcevec force;
		double forceVecAng;
	double polypos[2];
	double polyvel[2];
	double polyang;
	double polyangvel;
	double boundpos[2];
	double boundvel[2];

	double delta_t, radius;
	int sides;
		
	
};
#endif
