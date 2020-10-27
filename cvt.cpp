#include "cvt.h"

#include <iostream>

#include "LBFGSB.h"

inline float jcv_det(const jcv_point &a, const jcv_point &b) {
	return a.x * b.y - a.y * b.x;
}
inline jcv_point operator-(const jcv_point &a, const jcv_point &b) {
	return {a.x - b.x, a.y - b.y};
}
inline jcv_point operator+(const jcv_point &a, const jcv_point &b) {
	return {a.x + b.x, a.y + b.y};
}
inline jcv_point& operator+=(jcv_point &a, const jcv_point &b) {
	a.x += b.x;
	a.y += b.y;
	return a;
}
inline jcv_point operator/(const jcv_point &a, float b) {
	return {a.x/b, a.y/b};
}
inline jcv_point operator*(float a, const jcv_point &b) {
	return {a*b.x, a*b.y};
}

float CVT::operator()(const Eigen::VectorXf &x, Eigen::VectorXf &grad) {
	// Create array of points
	uint N = 0;
	for(const Object &o : *objs) N += o.size(); 
	jcv_point *points = new jcv_point[N];
	std::vector<uint> from(N);
	uint j = 0;
	for(uint i = 0; i < objs->size(); ++i) {
		uint n = (*objs)[i].size();
		for(uint k = 0; k < n; ++k) {
			from[j] = i;
			points[j].x = x(2*i) + (*objs)[i].getVertices()[2*k];
			points[j++].y = x(2*i+1) + (*objs)[i].getVertices()[2*k+1];
		}
	}

	// Compute diagram
	jcv_diagram diagram;
	memset(&diagram, 0, sizeof(jcv_diagram));
	jcv_diagram_generate(N, points, &box, nullptr, &diagram);
	const jcv_site* sites = jcv_diagram_get_sites(&diagram);

	// computation
	float fx = 0;
	grad.setZero();
	for(int i0 = 0; i0 < diagram.numsites; ++i0) {
		const jcv_site *site = &sites[i0];
		const jcv_graphedge *e = site->edges;
		const int i = from[site->index];
		const jcv_point &c = site->p;
		// moment of order 0
		float area = 0;
		// momemts of order 1 and 2
		jcv_point m = {0, 0}, m2 = {0, 0};
		while(e) {
			float a = .5 * abs(jcv_det(e->pos[0]-c, e->pos[1]-c));
			m += a * (c + e->pos[0] + e->pos[1]) / 3.;
			m2.x += a / 6. * (c.x*(c.x+e->pos[0].x+e->pos[1].x) + e->pos[0].x*(e->pos[0].x+e->pos[1].x) + e->pos[1].x*e->pos[1].x);
			m2.y += a / 6. * (c.y*(c.y+e->pos[0].y+e->pos[1].y) + e->pos[0].y*(e->pos[0].y+e->pos[1].y) + e->pos[1].y*e->pos[1].y);
			area += a;
			e = e->next;
		}
		const jcv_point C = {x(2*i), x(2*i+1)};
		fx += m2.x - 2*m.x*C.x + area*C.x*C.x;
		fx += m2.y - 2*m.y*C.y + area*C.y*C.y;
		grad(2*i) += 2 * (area * C.x - m.x);
		grad(2*i+1) += 2 * (area * C.y - m.y);
	}

	// free
	jcv_diagram_free(&diagram);
	delete[] points;

	return fx;
}

void CVT::solve() {
	// Solver
	LBFGSpp::LBFGSBParam<float> param;
	param.epsilon = 1e-5;
	param.max_iterations = 13;
	LBFGSpp::LBFGSBSolver<float> solver(param);

	// Bounds
	uint N = objs->size()*2;
	Eigen::VectorXf lb(N), ub(N);
	for(uint i = 0; i < N; ++i) {
		float m = 1e9;
		for(uint j = 0; j < (*objs)[i/2].size(); ++j) m = std::min(m, (*objs)[i/2].getVertices()[2*j+(i&1)]);
		lb(i) = ((i&1) ? box.min.y : box.min.x) - m;
	}
	for(uint i = 0; i < N; ++i) {
		float m = -1e9;
		for(uint j = 0; j < (*objs)[i/2].size(); ++j) m = std::max(m, (*objs)[i/2].getVertices()[2*j+(i&1)]);
		ub(i) = ((i&1) ? box.max.y : box.max.x) - m;
	}

	// start
	Eigen::VectorXf x(N);
	for(uint i = 0; i < N; ++i) x(i) = (*objs)[i/2].center[i&1];

	// Minimize
	float fx;
	solver.minimize(*this, x, fx, lb, ub);

	// Update centers
	for(uint i = 0; i < N; ++i) (*objs)[i/2].center[i&1] = x(i);
}

void CVT::getCells(std::vector<Object> &cells) const {
	// Create array of points
	uint N = 0;
	for(const Object &o : *objs) N += o.size(); 
	jcv_point *points = new jcv_point[N];
	std::vector<uint> from(N);
	uint j = 0;
	for(uint i = 0; i < objs->size(); ++i) {
		uint n = (*objs)[i].size();
		for(uint k = 0; k < n; ++k) {
			from[j] = i;
			points[j].x = (*objs)[i].center[0] + (*objs)[i].getVertices()[2*k];
			points[j++].y = (*objs)[i].center[1] + (*objs)[i].getVertices()[2*k+1];
		}
	}

	// Compute diagram
	jcv_diagram diagram;
	memset(&diagram, 0, sizeof(jcv_diagram));
	jcv_diagram_generate(N, points, &box, nullptr, &diagram);
	const jcv_site* sites = jcv_diagram_get_sites(&diagram);

	// add cells
	std::vector<std::vector<float>> vertices(objs->size());
	std::vector<std::vector<uint>> indices(objs->size());
	for(int i0 = 0; i0 < diagram.numsites; ++i0) {
		const jcv_site *site = &sites[i0];
		const jcv_graphedge *e = site->edges;
		const int i = from[site->index];
		uint s = vertices[i].size()/2;
		vertices[i].push_back(site->p.x);
		vertices[i].push_back(site->p.y);
		while(e) {
			uint t = vertices[i].size()/2;
			vertices[i].push_back(e->pos[0].x);
			vertices[i].push_back(e->pos[0].y);
			vertices[i].push_back(e->pos[1].x);
			vertices[i].push_back(e->pos[1].y);
			indices[i].push_back(s);
			indices[i].push_back(t);
			indices[i].push_back(t+1);
			e = e->next;
		}
	}
	for(uint i = 0; i < objs->size(); ++i) {
		float col[3] = {(*objs)[i].color[0]*.7f, (*objs)[i].color[1]*.7f, (*objs)[i].color[2]*.7f};
		cells.emplace_back(vertices[i], indices[i], col);
	}

	// free
	jcv_diagram_free(&diagram);
}