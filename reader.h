#pragma once

#include <istream>

struct Object {
	uint VAO, VBO, EBO;
	size_t size;
};

Object readXNC(std::istream &in);
Object readGerber(std::istream &in);