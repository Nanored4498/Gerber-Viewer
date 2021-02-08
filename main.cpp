#include <iostream>
#include <fstream>
#include <vector>
#include <random>

#include "glad.h"
#include <GLFW/glfw3.h>

#include "reader.h"
#include "cvt.h"

using namespace std;

const int WIDTH = 1024, HEIGHT = 768;
float X0=1e9, X1=-1e9, Y0=1e9, Y1=-1e9;
float cX, cY, pressX, pressY;
bool left_pressed = false;
float zoom = 1.f, ratio = 1.f;
uint shaderProgram;
default_random_engine re;
uniform_real_distribution<float> unif(0., 1.);

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
	ratio = (float) width / (float) height;
	glUniform1f(glGetUniformLocation(shaderProgram, "ratio"), ratio);
	glViewport(0, 0, width, height);
}

void getCoord(GLFWwindow* window, float &x, float &y) {
	int W, H;
	glfwGetWindowSize(window, &W, &H);
	double x0, y0;
	glfwGetCursorPos(window, &x0, &y0);
	x = cX + (2.f*x0/W - 1.f) * zoom;
	y = cY + (1.f - 2.f*y0/H) * zoom / ratio;
}

void mouseButtonCallBack(GLFWwindow* window, int button, int action, int mods) {
	if(button == GLFW_MOUSE_BUTTON_LEFT) {
		if(action == GLFW_PRESS) {
			left_pressed = true;
			getCoord(window, pressX, pressY);
		} else left_pressed = false;
	}
}

void cursorPositionCallback(GLFWwindow* window, double x, double y) {
	if(!left_pressed) return;
	float x2, y2;
	getCoord(window, x2, y2);
	cX += pressX - x2;
	cY += pressY - y2;
	glUniform2f(glGetUniformLocation(shaderProgram, "center"), cX, cY);
}

void scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
	zoom *= std::pow(1.1, -yoffset);
	glUniform1f(glGetUniformLocation(shaderProgram, "zoom"), zoom);
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
    	glfwTerminate();
		return 1;
	}
	int succes;
	char infoLog[512];
	
	// Vertex shader
	uint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	const char *vertexShaderSource =
		"#version 330 core\n"
		"layout (location = 0) in vec2 pos;\n"
		"uniform vec2 center;\n"
		"uniform float ratio;\n"
		"uniform float zoom;\n"
		"uniform vec2 trans;\n"
		"void main() {\n"
		"	vec2 n_pos = trans + pos - center;\n"
		"	gl_Position = vec4(n_pos.x, n_pos.y * ratio, 0., zoom);\n"
		"}";
	glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
	glCompileShader(vertexShader);
	glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &succes);
	if(!succes) {
		glGetShaderInfoLog(vertexShader, sizeof(infoLog), nullptr, infoLog);
		cerr << "Failed to compile vertex shader:\n" << infoLog << endl;
    	glfwTerminate();
		return 1;
	}
	
	// Fragment shader
	uint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
	const char *fragmentShaderSource =
		"#version 330 core\n"
		"out vec4 fragColor;\n"
		"uniform vec3 color;\n"
		"void main() {\n"
		"	fragColor = vec4(color, 1.0);\n"
		"}";
	glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
	glCompileShader(fragmentShader);
	glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &succes);
	if(!succes) {
		glGetShaderInfoLog(fragmentShader, sizeof(infoLog), nullptr, infoLog);
		cerr << "Failed to compile fragment shader:\n" << infoLog << endl;
    	glfwTerminate();
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
    	glfwTerminate();
		return 1;
	}
	glDetachShader(shaderProgram, vertexShader);
	glDetachShader(shaderProgram, fragmentShader);
	glDeleteShader(vertexShader);
	glDeleteShader(fragmentShader);
	glUseProgram(shaderProgram);
	int transLocation = glGetUniformLocation(shaderProgram, "trans");
	int colorLocation = glGetUniformLocation(shaderProgram, "color");

	// Read input files
	float drillColor[3] = {.2, .2, .8};
	PCB pcb;
	for(int i = 1; i < argc; ++i) {
		string path = argv[i];
		ifstream in(path);
		if(in.is_open()) {
			if(path.substr(path.size()-3) == "drl") readXNC(in, drillColor, pcb);
			else {
				uint start = pcb.objs.size();
				readGerber(in, drillColor, pcb);
				for(uint j = start; j < pcb.objs.size(); ++j)
					for(uint c = 0; c < 3; ++c)
						pcb.objs[j].color[c] = unif(re);
			}
			in.close();
		} else cerr << "Can't open file: " << argv[i] << endl;
	}
	for(const Object &o : pcb.objs) o.update_bounding_box(X0, X1, Y0, Y1);

	framebufferSizeCallback(window, WIDTH, HEIGHT);
	cX = (X0+X1)/2.f;
	cY = (Y0+Y1)/2.f;
	glUniform2f(glGetUniformLocation(shaderProgram, "center"), cX, cY);
	zoom = max(X1-X0, (Y1-Y0)*ratio) / 1.8f;
	glUniform1f(glGetUniformLocation(shaderProgram, "zoom"), zoom);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	glfwSetCursorPosCallback(window, cursorPositionCallback);
	glfwSetMouseButtonCallback(window, mouseButtonCallBack);
	glfwSetScrollCallback(window, scrollCallback);
	glClearColor(0.16, 0.2, 0.16, 1.0);
	// glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

	// CVT ???
	jcv_rect rect = {{X0 - .05f*(X1-X0), Y0 - .05f*(Y1-Y0)},
					{X1 + .05f*(X1-X0), Y1 + .05f*(Y1-Y0)}};
	CVT cvt(&pcb, rect);
	cvt.solve();
	vector<Object> cells;
	cvt.getCells(cells);
	pcb.computeEdgeObjects();

	// Rendering loop
	while(!glfwWindowShouldClose(window)) {
		glClear(GL_COLOR_BUFFER_BIT);
		for(const Object &o : cells) o.render(transLocation, colorLocation);
		pcb.render(transLocation, colorLocation);
		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	pcb.clear();
	for(Object &o : cells) o.deleteBuffers();
	glDeleteProgram(shaderProgram);
	glfwTerminate();

	return 0;
}