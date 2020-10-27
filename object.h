#pragma once

#include <vector>
#include <string>

#include "glad.h"

typedef unsigned int uint;

struct Aperture {
	std::string temp_name;
	std::vector<double> parameters;
};

class Object {
public:
	Object(std::vector<float> &vertices, const std::vector<uint> &indices, const float colors[3]);

	void render(int center_uniform, int color_uniform) const;

	void update_bounding_box(float &x0, float &x1, float &y0, float &y1) const;

	void deleteBuffers() {
		glDeleteVertexArrays(1, &VAO);
		glDeleteBuffers(1, &VBO);
		glDeleteBuffers(1, &EBO);
	}

	const std::vector<float>& getVertices() const { return vertices; }
	size_t size() const { return vertices.size()/2; }

	float center[2];
	float color[3];

private:
	std::vector<float> vertices;
	uint VAO, VBO, EBO;
	size_t indices_size;
};

class Path {
public:
	Path(Object &p_a, Object &p_b, const std::vector<float> &p_inter, const Aperture &p_ap, int p_interpolation_mode=1):
		a(p_a), b(p_b), inter(p_inter), ap(p_ap), interpolation_mode(p_interpolation_mode) {}

	Object getObject() const;

private:
	Object &a, &b;
	std::vector<float> inter;
	Aperture ap;
	int interpolation_mode;
};