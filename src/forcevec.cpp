#include "forcevec.h"

GLuint Forcevec::modelLoc;

Forcevec::Forcevec(double radius){
	
	vertices.push_back(0);
	vertices.push_back(0.05);
	vertices.push_back(0);

	vertices.push_back(0);
	vertices.push_back(-0.05);
	vertices.push_back(0);
	

	vertices.push_back(8.6);
	vertices.push_back(-0.05);
	vertices.push_back(0);

	vertices.push_back(8.6);
	vertices.push_back(0.05);
	vertices.push_back(0);

	indices.push_back(0);
	indices.push_back(1);
	indices.push_back(2);
	indices.push_back(2);
	indices.push_back(3);
	indices.push_back(0);

	glBufferData(GL_ARRAY_BUFFER, vertices.size()*sizeof(GLfloat), &vertices[0], GL_STATIC_DRAW);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size()*sizeof(GLushort), &indices[0], GL_STATIC_DRAW);

}


void Forcevec::draw(double a, double b, float ang){//, GLfloat c){
	
	glm::mat4 transform;

	transform = glm::translate(transform, glm::vec3(a,b,0));
		
	transform = glm::rotate(transform, ang, glm::vec3(0.0, 0.0, 1.0));
glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(transform));
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_SHORT, (GLvoid*) 0);
}
