#ifndef  _POLYGON_H_    /* only process this file once */
#define  _POLYGON_H_
#define GLM_FORCE_RADIANS 1
#include <iostream>
#include <vector>
#include <cmath>
#include "glew.h"
#include <GLFW/glfw3.h>
#include "glm/glm.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/type_ptr.hpp"


class Polygon{
	
public:
	std::vector<GLfloat> vertices;
	
	std::vector<GLushort> indices;
	
	static GLuint modelLoc;
	Polygon(){}
	Polygon(double r, int sides);
	void draw(GLfloat a, GLfloat b, GLfloat c, float ang);
private:
	double radius;
};


#endif
