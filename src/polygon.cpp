#include "polygon.h"

GLuint Polygon::modelLoc;

Polygon::Polygon(double radius, int sides){
	for(int i=0; i<3; i++) vertices.push_back(0);
	
	for(int i=0; i<sides; i++){
		vertices.push_back( radius*cos((M_PI*i) / (sides/2.0)));
		vertices.push_back( radius*sin((M_PI*i) /(sides/2.0)));
		vertices.push_back(0);
	}
	
	for(int i=1; i<sides; i++){
		indices.push_back(0);
		indices.push_back(i);
		indices.push_back(i+1);
	}
	indices.push_back(0);
	indices.push_back(sides);
	indices.push_back(1);
	glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(GLfloat), &vertices[0], GL_STATIC_DRAW);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLushort), &indices[0], GL_STATIC_DRAW);
}


void Polygon::draw(GLfloat a, GLfloat b, GLfloat c, float ang){//, GLfloat c){
	
	glm::mat4 transform;
	
	transform = glm::translate(transform, glm::vec3(a,b,c));
	transform = glm::rotate(transform, ang, glm::vec3(0.0, 0.0, 1.0));
	glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(transform));
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, (GLvoid*) 0);
}
