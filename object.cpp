#include "object.h"

using namespace std;

Object::Object(vector<float> &p_vertices, vector<uint> &p_indices, const float p_color[3]) {
	center[0] = 0;
	center[1] = 0;
	vertices = p_vertices;
	for(uint i = 0; i < vertices.size(); ++i) center[i&1] += vertices[i];
	center[0] *= 2. / vertices.size();
	center[1] *= 2. / vertices.size();
	for(uint i = 0; i < vertices.size(); ++i) vertices[i] -= center[i&1];
	indices = p_indices;
	for(int i = 0; i < 3; ++i) color[i] = p_color[i];

	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*indices.size(), indices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Object::render(int center_uniform, int color_uniform) const {
	glBindVertexArray(VAO);
	glUniform2f(center_uniform, center[0], center[1]);
	glUniform3f(color_uniform, color[0], color[1], color[2]);
	glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);
	glBindVertexArray(0);
}

void Object::update_bounding_box(float &x0, float &x1, float &y0, float &y1) const {
	for(int i = 0; i < (int) vertices.size(); i += 2) {
		x0 = min(x0, center[0] + vertices[i]);
		x1 = max(x1, center[0] + vertices[i]);
		y0 = min(y0, center[1] + vertices[i+1]);
		y1 = max(y1, center[1] + vertices[i+1]);
	}
}