
#ifndef  _ANIMATION_H_    /* only process this file once */
#define  _ANIMATION_H_

#define GLEW_STATIC
#define DELTA_T 0.000001
#include <iostream>
#include "circle.h"
#include "polygon.h"
#include <mutex>
#include <atomic>
#include <unistd.h>

class Animation{
	public:
	
		Animation(){  }
		Animation(int n){
			notReady = true;
			drawing = false;
			boundpos[0] = 0;
			boundpos[1] = 0;
		}
		void initialize();
		void setup();
		void compileShaders();
		void generateBuffers();
		void generateShapes();
		void setProjectionMatrices();
		void draw();
		void drawShapes();
		void setDisks(double* pp, double* pv, double pa, double pav, double* b, double* v);
		void moveDisks(double time);
		std::atomic<bool> notReady;
	std::atomic<bool> drawing;
	
	private:
		GLuint s_VBO, s_VAO, s_EBO, shaderProgram, modelLoc, colorLoc, viewLoc;
		GLuint b_VBO, b_VAO, b_EBO;
		int width, height;
		GLFWwindow* window;
		Polygon polygon;
		Circle bound;
	double polypos[2];
	double polyvel[2];
	double polyang;
	double polyangvel;
	double boundpos[2];
	double boundvel[2];
		
	
};
#endif
