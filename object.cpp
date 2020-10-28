#include "object.h"

#include <cmath>

using namespace std;

Object::Object(vector<float> &p_vertices, const vector<uint> &p_indices, const float p_color[3]) {
	center[0] = 0;
	center[1] = 0;
	float area = 0.;
	vertices = p_vertices;
	indices_size = p_indices.size();
	for(uint i = 0; i < indices_size; i += 3) {
		uint j = 2*p_indices[i], k = 2*p_indices[i+1], l = 2*p_indices[i+2];
		float ux = vertices[k] - vertices[j];
		float uy = vertices[k+1] - vertices[j+1];
		float vx = vertices[l] - vertices[j];
		float vy = vertices[l+1] - vertices[j+1];
		float a = .5 * abs(ux*vy - uy*vx);
		center[0] += a * (vertices[j] + vertices[k] + vertices[l]) / 3.;
		center[1] += a * (vertices[j+1] + vertices[k+1] + vertices[l+1]) / 3.;
		area += a;
	}
	center[0] /= area;
	center[1] /= area;
	for(uint i = 0; i < vertices.size(); ++i) vertices[i] -= center[i&1];
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
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*indices_size, p_indices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void Object::render(int center_uniform, int color_uniform) const {
	glBindVertexArray(VAO);
	glUniform2f(center_uniform, center[0], center[1]);
	glUniform3f(color_uniform, color[0], color[1], color[2]);
	glDrawElements(GL_TRIANGLES, indices_size, GL_UNSIGNED_INT, 0);
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

void PCB::computeEdgeObjects() {
	edges_obj.clear();
	float color[3] = {.33, .33, .33};
	for(const PCBEdge &e : edges) {
		if(apertures[e.ap_id].temp_name == "C") {
			if(e.interpolation_mode == 1) {
				vector<float> vertices;
				vector<uint> indices;
				double r = .5*apertures[e.ap_id].parameters[0];
				double xs[2], ys[2];
				xs[0] = e.from >= 0 ? objs[e.from].center[0] : junctions[-e.from-1];
				ys[0] = e.from >= 0 ? objs[e.from].center[1] : junctions[-e.from];
				xs[1] = e.to >= 0 ? objs[e.to].center[0] : junctions[-e.to-1];
				ys[1] = e.to >= 0 ? objs[e.to].center[1] : junctions[-e.to];
				double a0 = atan2(ys[1]-ys[0], xs[1]-xs[0]);
				for(int j : {0, 1}) {
					uint ind = vertices.size() / 2;
					for(int i = 0; i <= 12; ++i) {
						double a = (2*M_PI*i)/24 + M_PI_2 + a0 + j*M_PI;
						vertices.push_back(xs[j] + r*cos(a));
						vertices.push_back(ys[j] + r*sin(a));
						if(i < 12) {
							indices.push_back(ind + i);
							indices.push_back(ind + i+1);
							indices.push_back(ind + 13);
						}
					}
					vertices.push_back(xs[j]);
					vertices.push_back(ys[j]);
				}
				indices.push_back(0);
				indices.push_back(12);
				indices.push_back(14);
				indices.push_back(0);
				indices.push_back(14);
				indices.push_back(26);
				edges_obj.emplace_back(vertices, indices, color);
			}
		}
	}
}