
#include "animation.h"

std::mutex lock;



const GLuint WIDTH = 800, HEIGHT = 600;

// Shaders
const GLchar* vertexShaderSource =
"#version 330 core\n"
"layout (location = 0) in vec3 position;\n"
"out vec3 fragPos;\n"
"uniform mat4 model;\n"
"uniform mat4 view;\n"
"uniform mat4 projection;\n"
"void main()\n"
"{\n"
	"fragPos = vec3(model * vec4(position, 1.0));\n"
	"gl_Position = projection*view*vec4(fragPos,1.0f);\n"
"}\0";

const GLchar* fragmentShaderSource = "#version 330 core\n"
"in vec3 fragPos;\n"
"out vec4 color;\n"
"uniform vec4 ourColor;\n"
"void main()\n"
"{\n"
"	color = ourColor;\n"
"}\n\0";

void Animation::initialize(){
	//Initialize GLFW
	if (!glfwInit())
	return;
	//Set GLFW settings
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	
	/* Create a windowed mode window and its OpenGL context */
	window = glfwCreateWindow(WIDTH, HEIGHT, "Window", NULL, NULL);
	if (!window)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	
	/* Make the window's context current */
	glfwMakeContextCurrent(window);
	
	// Set this to true so GLEW knows to use a modern approach to retrieving function pointers and extensions
	glewExperimental = GL_TRUE;
	// Initialize GLEW to setup the OpenGL Function pointers
	glewInit();
	
	
	glfwGetFramebufferSize(window, &width, &height);
	glViewport(0, 0, width, height);
	
	glEnable(GL_DEPTH_TEST);
	
}

//The following function can be simplified with the introduction of a shader.cpp file
void Animation::compileShaders(){
	// Build and compile our shader program
	// Vertex shader
	GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
	glCompileShader(vertexShader);
	
	// Check for compile time errors
	GLint success;
	GLchar infoLog[512];
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
	}
	// Fragment shader
	GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
	glCompileShader(fragmentShader);
	// Check for compile time errors
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
	if (!success)
	{
		glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
	}
	// Link shaders
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	// Check for linking errors
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
	if (!success) {
		glGetProgramInfoLog(shaderProgram, 512, NULL, infoLog);
		std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
	}
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	
	//Use the newly linked shader program
	glUseProgram(shaderProgram);
}



void Animation::setup(){
	initialize();
	compileShaders();
	generateBuffers();
	generateShapes();
	setProjectionMatrices();
}

void Animation::generateBuffers(){
	//	We use three buffers.
	//	VBO (vertex buffer object) contains the vertex positions
	//	EBO (element buffer object) contains the indices of the vertices (aka the order to draw triangles)
	//	VAO (vertex attribute object) contains the alignment info of the data in the VBO
	glGenVertexArrays(1, &s_VAO);
	glGenBuffers(1, &s_VBO);
	glGenBuffers(1, &s_EBO);
	glGenVertexArrays(1, &b_VAO);
	glGenBuffers(1, &b_VBO);
	glGenBuffers(1, &b_EBO);	
	glGenVertexArrays(1, &f_VAO);
	glGenBuffers(1, &f_VBO);
	glGenBuffers(1, &f_EBO);

	// Bind the Vertex Array Object first, then bind and set vertex buffer(s) and attribute pointer(s).
	glBindVertexArray(s_VAO);

	glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, s_EBO);
	

	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind VAO
	glBindVertexArray(0);
	
	glBindVertexArray(b_VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, b_VBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, b_EBO);
	
	
	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind VAO
	glBindVertexArray(0);
	
	
	glBindVertexArray(f_VAO);
	
	glBindBuffer(GL_ARRAY_BUFFER, f_VBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, f_EBO);
	
	
	// Position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(GLfloat), (GLvoid*)0);
	glEnableVertexAttribArray(0);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind VAO
	glBindVertexArray(0);
	
	

	
	
	
	
	

}

void Animation::generateShapes(){
	//Bind VAO so sphere data can be transferred to VBO buffers on construction
	glBindVertexArray(s_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
	
	polygon = Polygon(radius, sides);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind
	glBindVertexArray(0);
	
	
	glBindVertexArray(b_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, b_VBO);
	circle = Circle(1);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind
	glBindVertexArray(0);
	

	
	glBindVertexArray(f_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, f_VBO);
	force = Forcevec(radius);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	//Unbind
	glBindVertexArray(0);
	
}

void Animation::setProjectionMatrices(){
	//This function creates 4x4 transformation matrices which are then passed to uniforms.
	//The uniforms are accessed by the shaderProgram, which then applies the matrices to the vertex positions
	
	//Initialize to identity
	glm::mat4 view;
	glm::mat4 projection;
	
	//View is at z = +8
	view = glm::translate(view, glm::vec3(0.0f, 0.0f, -30.0f));
//	view = glm::rotate(view, (float) -(M_PI/8.0), glm::vec3(0.0f, 1.0f, 0.0f));
	//Projection has 45 degree FoV, aspect ratio given by the window's, and
	//records only those objects within 0.1f and 100.0f of the "camera"
	projection = glm::perspective(glm::radians(45.0f), (GLfloat)WIDTH / (GLfloat)HEIGHT, 0.1f, 100.0f);
	
	// Get the locations of the shader's uniforms
	modelLoc = glGetUniformLocation(shaderProgram, "model");
	viewLoc = glGetUniformLocation(shaderProgram, "view");
	GLint projLoc = glGetUniformLocation(shaderProgram, "projection");
	colorLoc = glGetUniformLocation(shaderProgram, "ourColor");
	//Give the location of the model uniform to the Sphere class, so that translations can be applied
	//when Sphere::draw is called.
	Circle::modelLoc = modelLoc;
	Polygon::modelLoc = modelLoc;
	Forcevec::modelLoc = modelLoc;
	// Set the uniforms of the other matrices now - they won't change.
	glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
	glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
	
}

void Animation::draw(){
	//This loop runs until the window is closed. When this happens, this function reaches the last line, which results in the
	//entire program ending
	while(notReady);
	
	
	while (!glfwWindowShouldClose(window) )
	{
		/* Poll for and process events */
		glfwPollEvents();

		//Set background color
		glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		
		//Draw!
		drawShapes();
		
		/* Swap front and back buffers */
		glfwSwapBuffers(window);
	}
	
	//Cleanup
	glDeleteVertexArrays(1, &s_VAO);
	glDeleteBuffers(1, &s_VBO);
	glDeleteBuffers(1, &s_EBO);
	glfwTerminate();
	
}



void Animation::drawShapes(){
	
	lock.lock();
	


	if(drawForceVec){
		glBindVertexArray(f_VAO);
		glBindBuffer(GL_ARRAY_BUFFER, f_VBO);
		glUniform4f(colorLoc, 0.95f, 0.05f, 0.75f, 1.0f);
		force.draw(boundpos[0], boundpos[1], forceVecAng);
	
		glBindBuffer(GL_ARRAY_BUFFER, 0);
		glBindVertexArray(0);

	}


	glBindVertexArray(s_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, s_VBO);
	glUniform4f(colorLoc, 0.35f, 0.5f, 0.75f, 1.0f);
	polygon.draw(polypos[0], polypos[1], 0, polyang);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	

	glBindVertexArray(b_VAO);
	glBindBuffer(GL_ARRAY_BUFFER, b_VBO);
	glUniform4f(colorLoc, 0.8f, 0.8f, 0.8f, 1.0f);
	circle.draw(boundpos[0], boundpos[1],0, 8.6);

	circle.draw(boundpos[0], boundpos[1],0.1, 0.25);
	glUniform4f(colorLoc, 0.2f, 0.8f, 0.3f, 1.0f);
	//center circle
	circle.draw(polypos[0], polypos[1], 0.1, 0.25);
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	

	
	lock.unlock();
	
}



void Animation::setPoly(double* pp, double* pv, double pa, 
						double pav, double* b, double* v, double* v_pos){
	lock.lock();
	polyang = pa;
	double dif = pav - polyangvel;
	polyangvel = pav;

	for(int i=0; i<2; i++){
		polypos[i] = pp[i];
		polyvel[i] = pv[i];
		boundpos[i] = b[i];
		boundvel[i] = v[i];
	}
	lock.unlock();
	if(v_pos){

		if(dif>0){
			std::cout<<"Positive"<<std::endl;
		}else{
			std::cout<<"Negative"<<std::endl;
		}
		forceVecAng = atan2(v_pos[1],v_pos[0]);
		drawForceVec = true;
		sleep(2);
		drawForceVec = false;
	}
}

void Animation::movePoly(double time){
	double start = 0;
	while(start<time){
		lock.lock();
		for(int j=0; j<2; j++){
			polypos[j] += delta_t*polyvel[j];
		}
		polyang += delta_t*polyangvel;
		for(int j=0; j<2; j++) boundpos[j] += delta_t*boundvel[j];
		lock.unlock();
		start+= delta_t;
	}
	
	


}










