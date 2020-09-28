#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include <string>
#include <cmath>

#include "glad.h"
#include <GLFW/glfw3.h>

using namespace std;
typedef pair<uint, uint> puu;

const int WIDTH = 1024, HEIGHT = 768;

void framebufferSizeCallback(GLFWwindow* window, int width, int height) {
	glViewport(0, 0, width, height);
}

void skipLine(istream &in, char end='\n') {
	char c='0';
	while(c != end && c != EOF) in.get(c);
}

struct Object {
	uint VAO, VBO, EBO;
	size_t size;
};

Object readXNC(istream &in) {
	float SCALE = 1.0;
	float tools[100];
	for(int t = 0; t < 100; ++t) tools[t] = -1;
	bool incremental = false;
	bool drill = true;
	vector<float> vertices;
	vector<uint> indices;

	// header
	string s;
	in >> s;
	if(s == "M48") {
		while(true) {
			in >> s;
			if(s == "%") break;
			else if(s == ";") skipLine(in);
			else if(s == "METRIC") SCALE = 1.0;
			else if(s == "INCH") SCALE = 25.4;
			else if(s[0] == 'T') {
				size_t i = s.find('C');
				if(i == string::npos) {
					cerr << "Unknown word: " << s << endl;
					skipLine(in);
				} else {
					int t = stoi(s.substr(1, i-1));
					if(t < 0 || t >= 100) cerr << "Tool index has to be between 0 and 99" << endl;
					else tools[t] = stof(s.substr(i+1)) * SCALE;
				}
			} else {
				cerr << "Unknown word: " << s << endl;
				skipLine(in);
			} 
		}
		in >> s;
	}

	// body
	int T = -1;
	while(s != "M30") {
		if(s == "G90") incremental = false;
		else if(s == "G91") incremental = true;
		else if(s == "G00") drill = false;
		else if(s == "G05") drill = true;
		else if(s[0] == 'T') {
			int t = atoi(s.c_str()+1);
			if(t < 0 || t >= 100) cerr << "Tool index has to be between 0 and 99 (you gave: " << t << ")" << endl;
			else if(tools[t] < 0) cerr << "The tool " << t << " is not defined" << endl;
			else T = t;
		} else if(s[0] == 'X') {
			size_t i = s.find('Y');
			if(i == string::npos) {
				cerr << "Unknown word: " << s << endl;
				skipLine(in);
			} else if(!drill) cerr << "Drill mode required for the command: " << s << endl;
			else if(T < 0) cerr << "No tool selected while executing the command: " << s << endl;
			else {
				float x = stof(s.substr(1, i-1)) * SCALE;
				float y = stof(s.substr(i+1)) * SCALE;
				float d = tools[T];
				uint ind = vertices.size() / 3;
				for(int i = 0; i < 24; ++i) {
					float a = 2*M_PI*i/24;
					vertices.push_back(x + d*cos(a));
					vertices.push_back(y + d*sin(a));
					vertices.push_back(0);
					indices.push_back(ind + i);
					indices.push_back(ind + ((i+1) % 24));
					indices.push_back(ind + 24);
				}
				vertices.push_back(x);
				vertices.push_back(y);
				vertices.push_back(0);
			}
		} else {
			cerr << "Unknown word: " << s << endl;
			skipLine(in);
		}
		in >> s;
	}

	uint VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*indices.size(), indices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	return {VAO, VBO, EBO, indices.size()};
}

struct Aperture {
	string temp_name;
	vector<double> parameters;
};

Object readGerber(istream &in) {
	double SCALE = 1.0;
	bool dark = true;
	map<int, Aperture> apertures;
	int ap_id = -1;
	bool in_region = false;
	bool circular = false, clockwise;
	double cX, cY;
	vector<float> vertices;
	vector<uint> indices;

	string s;
	in >> s;
	while(s != "M02*") {
		if(s == "G04") skipLine(in, '*');
		else if(s.size() == 13 && s.substr(0, 6) == "%FSLAX" && s[8] == 'Y' && s[11] == '*' && s[12] == '%') {
			// this command is not very important
		} else if(s.size() == 7 && s.substr(0, 3) == "%MO" && s[5] == '*' && s[6] == '%') {
			string unit = s.substr(3, 2);
			if(unit == "MM") SCALE = 1.0;
			else if(unit == "IN") SCALE = 25.4;
			else cerr << "Unknown unit: " << unit << endl;
		} else if(s.size() == 6 && s.substr(0, 3) == "%LP" && s[4] == '*' && s[5] == '%') {
			char polarity = s[3];
			if(polarity == 'D') dark = true;
			else if(polarity == 'C') {
				dark = false;
				cerr << "Warning: Clear polarity mode is not implemented !!" << endl;
			} else cerr << "Unknown polarity: " << polarity << endl;
		} else if(s.substr(0, 4) == "%ADD" && s[s.size()-2] == '*' && s.back() == '%') {
			size_t temp_ind = 4;
			while(s[temp_ind] >= '0' && s[temp_ind] <= '9') ++ temp_ind;
			int aper_id = stoi(s.substr(4, temp_ind-4));
			if(aper_id < 10) cerr << "The aperture number " << aper_id << " is reserved and cannot be used" << endl;
			else {
				size_t param_ind = temp_ind;
				while(s[param_ind] != ',' && s[param_ind] != '*') ++ param_ind;
				Aperture ap = {s.substr(temp_ind, param_ind-temp_ind), {}};
				while(s[param_ind] != '*') {
					size_t start = ++ param_ind;
					while(s[param_ind] != 'X' && s[param_ind] != '*') ++ param_ind;
					ap.parameters.push_back(stod(s.substr(start, param_ind-start)));
				}
				apertures[aper_id] = ap;
			}
		} else if(s[0] == 'D' && s.back() == '*') {
			int id = atoi(s.substr(1, s.size()-2).c_str());
			if(apertures.count(id)) ap_id = id;
			else cerr << "The aperture number " << id << " is not defined" << endl;
		} else if(s == "G01*") circular = false;
		else if(s == "G02*") {
			circular = true;
			clockwise = true;
		} else if(s == "G03*") {
			circular = true;
			clockwise = false;
		} else if(s == "G36*") in_region = true;
		else if(s == "G37*") in_region = false;
		else if(s.size() >= 8 && s[0] == 'X' && s[s.size()-4] == 'D' && s[s.size()-3] == '0' && s.back() == '*') {
			size_t y_ind = s.find('Y');
			double x = stod(s.substr(1, y_ind-1));
			double y = stod(s.substr(y_ind+1, s.size()-5-y_ind));
			char d = s[s.size()-2];
			if(d == '1') {
				// interpolation
			} else if(d == '3') {
				// flash
			} else if(d != '2') cerr << "Unknown operation: " << d << endl;
			cX = x;
			cY = y;
		} else cerr << "Unknown word: " << s << endl;
		in >> s;
	}

	uint VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*indices.size(), indices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	// glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	return {VAO, VBO, EBO, indices.size()};
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
	glViewport(0, 0, WIDTH, HEIGHT);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);
	glClearColor(0.2, 0.3, 0.3, 1.0);
	int succes;
	char infoLog[512];
	
	// Vertex shader
	uint vertexShader = glCreateShader(GL_VERTEX_SHADER);
	const char *vertexShaderSource =
		"#version 330 core\n"
		"layout (location = 0) in vec3 pos;\n"
		"void main() {\n"
		"	gl_Position = vec4((pos.x-135.)/40., (pos.y+110.)/30., 0., 1.);\n"
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
	uint shaderProgram = glCreateProgram();
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