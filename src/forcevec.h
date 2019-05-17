#ifndef  _FORCEVEC_H_    /* only process this file once */
#define  _FORCEVEC_H_
#define GLM_FORCE_RADIANS 1
#include <iostream>
#include <vector>
#include <cmath>
#include "glew.h"
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"

class Forcevec{
	
public:
	std::vector<GLfloat> vertices;
	
	std::vector<GLushort> indices;
	

	static GLuint modelLoc;
	Forcevec(){}
	Forcevec(double r);
	void draw( double a, double b, float ang);
private:
	double radius;
};


#endif