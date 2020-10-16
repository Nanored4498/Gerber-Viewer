#pragma once

#include <istream>

#include "glad.h"

struct Object {
	uint VAO, VBO, VBOc, EBO;
	size_t size;
	float x0, x1, y0, y1;
	void clean() {
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);
		glDeleteBuffers(1, &VBOc);
		glDeleteBuffers(1, &EBO);
	}
};

Object readXNC(std::istream &in, float *color0);
std::pair<Object, Object> readGerber(std::istream &in, float *color0);