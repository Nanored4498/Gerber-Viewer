#pragma once

#include <istream>

struct Object {
	uint VAO, VBO, EBO;
	size_t size;
	float x0, x1, y0, y1;
};

Object readXNC(std::istream &in);
Object readGerber(std::istream &in);