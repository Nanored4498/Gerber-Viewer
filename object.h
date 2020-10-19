#pragma once

#include <vector>

#include "glad.h"

typedef unsigned int uint;

class Object {
public:

	Object(std::vector<float> &vertices, std::vector<uint> &indices, float colors[3]);

	void render(int center_uniform, int color_uniform) const;

	void update_bounding_box(float &x0, float &x1, float &y0, float &y1) const;

	void deleteBuffers() {
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);
		glDeleteBuffers(1, &EBO);
	}

private:
	std::vector<float> vertices;
	std::vector<uint> indices;
	float center[2];
	float color[3];
	uint VAO, VBO, EBO;
};