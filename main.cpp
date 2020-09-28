#include <iostream>
#include <fstream>
#include <vector>

#include "glad.h"
#include <GLFW/glfw3.h>

#include "reader.h"

using namespace std;

const int WIDTH = 1024, HEIGHT = 768;
float x0=1e9, x1=-1e9, y0=1e9, y1=-1e9;
uint shaderProgram;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
	int centerLocation = glGetUniformLocation(shaderProgram, "center");
	int ratioLocation = glGetUniformLocation(shaderProgram, "ratio");
	int zoomLocation = glGetUniformLocation(shaderProgram, "zoom");
	float ratio = (float) width / (float) height;
	float zoom = max(x1-x0, (y1-y0)*ratio) / 1.8f;
	glUniform2f(centerLocation, (x0+x1)/2.f, (y0+y1)/2.f);
	glUniform1f(ratioLocation, ratio);
	glUniform1f(zoomLocation, zoom);
	glViewport(0, 0, width, height);
}

int main(int argc, char* argv[]) {

	// OpenGL Init
	if(!glfwInit()) {
		cerr << "Failed to initialize GLFW" << endl;
		return 1;
	}
	glfwWindowHint(GLFW_SAMPLES, 4); // Anti-alias x4
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // Setting version to 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	#ifdef __APPLE__
		glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	#endif
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // Using Core Profile
	GLFWwindow *window = glfwCreateWindow(WIDTH, HEIGHT, "Gerber Parser", nullptr, nullptr);
	if(window == nullptr){
    	cerr << "Failed to open GLFW window" << endl;
    	glfwTerminate();
    	return 1;
	}
	glfwMakeContextCurrent(window);
	if(!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
		cerr << "Failed to initialize GLAD" << endl;
		return 1;
	}
	int succes;
	char infoLog[512];
	
	// Vertex shader
	uint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	const char *vertexShaderSource =
		"#version 330 core\n"
		"layout (location = 0) in vec3 pos;\n"
		"uniform vec2 center;\n"
		"uniform float ratio;\n"
		"uniform float zoom;\n"
		"void main() {\n"
		"	gl_Position = vec4(pos.x-center.x, (pos.y-center.y) * ratio, 0., zoom);\n"
		"}";
	glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &succes);
	if(!succes) {
		glGetShaderInfoLog(vertexShader, sizeof(infoLog), nullptr, infoLog);
		cerr << "Failed to compile vertex shader:\n" << infoLog << endl;
		return 1;
	}
	
	// Fragment shader
	uint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	const char *fragmentShaderSource =
		"#version 330 core\n"
		"out vec4 fragColor;\n"
		"void main() {\n"
		"	fragColor = vec4(1.0, 0.5, 0.2, 1.0);\n"
		"}";
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
	glCompileShader(fragmentShader);
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &succes);
	if(!succes) {
		glGetShaderInfoLog(fragmentShader, sizeof(infoLog), nullptr, infoLog);
		cerr << "Failed to compile fragment shader:\n" << infoLog << endl;
		return 1;
	}

	// Shader program
	shaderProgram = glCreateProgram();
	glAttachShader(shaderProgram, vertexShader);
	glAttachShader(shaderProgram, fragmentShader);
	glLinkProgram(shaderProgram);
	glGetProgramiv(shaderProgram, GL_LINK_STATUS, &succes);
	if(!succes) {
		glGetProgramInfoLog(shaderProgram, sizeof(infoLog), nullptr, infoLog);
		cerr << "Failed to link shader program:\n" << infoLog << endl;
		return 1;
	}
	glDetachShader(shaderProgram, vertexShader);
	glDetachShader(shaderProgram, fragmentShader);
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	glUseProgram(shaderProgram);

	// Read input files
	vector<Object> objects;
	for(int i = 1; i < argc; ++i) {
		string path = argv[i];
		ifstream in(path);
		if(in.is_open()) {
			if(path.substr(path.size()-3) == "drl") objects.push_back(readXNC(in));
			else objects.push_back(readGerber(in));
			in.close();
		} else cerr << "Can't open file: " << argv[i] << endl;
	}
	for(const Object &o : objects) {
		x0 = min(x0, o.x0);
		x1 = max(x1, o.x1);
		y0 = min(y0, o.y0);
		y1 = max(y1, o.y1);
	}

	framebufferSizeCallback(window, WIDTH, HEIGHT);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	glClearColor(0.2, 0.3, 0.3, 1.0);

	// Rendering loop
	while(!glfwWindowShouldClose(window)) {
		glClear(GL_COLOR_BUFFER_BIT);
		for(const Object &o : objects) {
			glBindVertexArray(o.VAO);
			glDrawElements(GL_TRIANGLES, o.size, GL_UNSIGNED_INT, 0);
			glBindVertexArray(0);
		}
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	for(const Object &o : objects) {
		glDeleteVertexArrays(1, &o.VAO);
		glDeleteBuffers(1, &o.VBO);
		glDeleteBuffers(1, &o.EBO);
	}
	glDeleteProgram(shaderProgram);
	glfwTerminate();

	return 0;
}