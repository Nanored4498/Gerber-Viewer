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
	float getX(int i) const { return vertices[2*i]; }
	float getY(int i) const { return vertices[2*i+1]; }
	size_t size() const { return vertices.size()/2; }

	float center[2];
	float color[3];

private:
	std::vector<float> vertices;
	uint VAO, VBO, EBO;
	size_t indices_size;
};

struct PCBEdge {
	int from, to;
	int ap_id;
	int interpolation_mode;
	PCBEdge(int p_from, int p_to, int p_ap_id, int p_mode):
		from(p_from), to(p_to), ap_id(p_ap_id), interpolation_mode(p_mode) {}
};

class PCB {
public:
	void computeEdgeObjects();

	void render(int transLocation, int colorLocation) {
		for(const Object &o : objs) o.render(transLocation, colorLocation);
		for(const Object &o : edges_obj) o.render(transLocation, colorLocation);
	}

	void clear() {
		for(Object &o : objs) o.deleteBuffers();
		for(Object &o : edges_obj) o.deleteBuffers();
		objs.clear();
		apertures.clear();
		junctions.clear();
		edges.clear();
		edges_obj.clear();
	}

	std::vector<Object> objs;
	std::vector<Aperture> apertures;
	std::vector<float> junctions;
	std::vector<PCBEdge> edges;

private:
	std::vector<Object> edges_obj;
};