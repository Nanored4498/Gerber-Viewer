#include "cvt.h"

#include <iostream>
#include <random>

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
inline jcv_point& operator/=(jcv_point &a, float b) {
	a.x /= b;
	a.y /= b;
	return a;
}
inline jcv_point operator*(float a, const jcv_point &b) {
	return {a*b.x, a*b.y};
}
inline float jcv_norm(const jcv_point &a) {
	return std::sqrt(a.x*a.x + a.y*a.y);
}
inline float jcv_dot(const jcv_point &a, const jcv_point &b) {
	return a.x * b.x + a.y * b.y;
}
inline std::ostream& operator<<(std::ostream &s, const jcv_point &p) {
	return s << '(' << p.x << ' ' << p.y << ')';
}

#define STEPS 360.

float CVT::operator()(const Eigen::VectorXf &x, Eigen::VectorXf &grad) {

	float fx = 0;
	grad.setZero();
	float diag = jcv_norm(box.max - box.min);
	float tot_area = (box.max.x - box.min.x) * (box.max.y - box.min.y);
	float space = diag / STEPS;

	// Create array of points
	std::vector<jcv_point> points;
	std::vector<int> from;
	int no = pcb->objs.size() * 2;
	for(uint i = 0; i < pcb->objs.size(); ++i) {
		uint n = pcb->objs[i].size();
		for(uint k = 0; k < n; ++k) {
			from.push_back(2*i);
			points.emplace_back();
			points.back().x = x(2*i) + pcb->objs[i].getVertices()[2*k];
			points.back().y = x(2*i+1) + pcb->objs[i].getVertices()[2*k+1];
		}
	}
	for(int i = 0; i < (int) pcb->junctions.size(); i += 2) {
		from.push_back(no+i);
		points.push_back({x(no+i), x(no+i+1)});
	}
	// add edges
	for(const PCBEdge &e : pcb->edges) {
		// case 'C' and mode=1
		float r = .5*pcb->apertures[e.ap_id].parameters[0];
		int i0 = e.from < 0 ? no-e.from-1 : 2*e.from;
		int i1 = e.to < 0 ? no-e.to-1 : 2*e.to;
		jcv_point a = {x(i0), x(i0+1)}, b = {x(i1), x(i1+1)};
		jcv_point v = b-a;
		float d = jcv_norm(v);
		v /= d;
		jcv_point h = {-r*v.y, r*v.x}; // orthogonal vector
		int n = (d - space) / space; // number of intermediate seeds
		if(n&1) ++n;
		float sp = d / (n+1); // spacing between seeds
		// create seeds
		for(int i = 1; i <= n; ++i) {
			points.push_back(a + i*sp*v + h);
			from.push_back(2*i <= n ? i0 : i1);
			points.push_back(a + i*sp*v - h);
			from.push_back(2*i <= n ? i0 : i1);
		}
		// energy
		// orthogonal ditance
		h.x = e.from < 0 ? pcb->junctions[-e.from] : pcb->objs[e.from].center[1];
		h.x -= e.to < 0 ? pcb->junctions[-e.to] : pcb->objs[e.to].center[1];
		h.y = e.to < 0 ? pcb->junctions[-e.to-1] : pcb->objs[e.to].center[0];
		h.y -= e.from < 0 ? pcb->junctions[-e.from-1] : pcb->objs[e.from].center[0];
		float nh = jcv_norm(h);
		h /= nh;
		d = jcv_dot(h, b-a);
		fx += .5 * tot_area * d*d;
		jcv_point gr = tot_area * d * h;
		grad(i0) -= gr.x;
		grad(i0+1) -= gr.y;
		grad(i1) += gr.x;
		grad(i1+1) += gr.y;
		// dot with v
		// float coeff = tot_area * nh*nh;
		// v = {h.y, -h.x};
		// h = b-a;
		// d = jcv_norm(h);
		// h /= d;
		// float hv = jcv_dot(h, v);
		// float dif = hv - 1.;
		// fx += .5 * coeff * dif*dif;
		// jcv_point gr = coeff * dif * (v - hv*h) / d;
		// if(e.from < 0) { grad(no-e.from-1) -= gr.x; grad(no-e.from) -= gr.y; }
		// else { grad(2*e.from) -= gr.x; grad(2*e.from+1) -= gr.y; }
		// if(e.to < 0) { grad(no-e.to-1) += gr.x; grad(no-e.to) += gr.y; }
		// else { grad(2*e.to) += gr.x; grad(2*e.to+1) += gr.y; }

	}
	int N = points.size();
	// for(int i = 0; i < N; ++i) {
	// 	for(int j = 0; j < i; ++j) {
	// 		if(points[i].x == points[j].x && points[i].y == points[j].y) {
	// 			std::cerr << "AAAIIIIEEEE !!!!!! " << i << " " << j << " " << from[i] << " " << from[j]
	// 					<< " " << pcb->objs[from[i]].size() << " " << pcb->objs[from[j]].size() << std::endl;
	// 		}
	// 	}
	// }

	// Compute diagram
	jcv_diagram diagram;
	memset(&diagram, 0, sizeof(jcv_diagram));
	jcv_diagram_generate(N, points.data(), &box, nullptr, &diagram);
	const jcv_site* sites = jcv_diagram_get_sites(&diagram);

	// computation
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
			m += a / 3. * (c + e->pos[0] + e->pos[1]);
			m2.x += a / 6. * (c.x*(c.x+e->pos[0].x+e->pos[1].x) + e->pos[0].x*(e->pos[0].x+e->pos[1].x) + e->pos[1].x*e->pos[1].x);
			m2.y += a / 6. * (c.y*(c.y+e->pos[0].y+e->pos[1].y) + e->pos[0].y*(e->pos[0].y+e->pos[1].y) + e->pos[1].y*e->pos[1].y);
			area += a;
			e = e->next;
		}
		jcv_point C = {x(i), x(i+1)};
		fx += m2.x - 2.*m.x*C.x + area*C.x*C.x;
		fx += m2.y - 2.*m.y*C.y + area*C.y*C.y;
		if(std::abs(area*C.x - m.x) > 1e3) {
			std::cerr << "BUUUUUUUUUUUUUUUG " << area << " " << tot_area << std::endl;
			std::cerr << C << " " << c << " " << box.min << " " << box.max << std::endl;
			e = site->edges;
			while(e) {
				std::cerr << ' ' << e->pos[0] << ' ' << e->pos[1];
				e = e->next;
			}
			std::cerr << std::endl << box.min << " " << box.max << " " << std::endl;
		}
		grad(i) += 2. * (area*C.x - m.x);
		grad(i+1) += 2. * (area*C.y - m.y);
	}

	// free
	jcv_diagram_free(&diagram);

	std::cerr << std::fixed << fx << std::endl;
	// std::cerr << grad << std::endl;
	if(fx > 1e6) {
		grad.setZero();
		return 0;
		// exit(1);
	}
	return fx;
}

void CVT::solve() {
	// Solver
	LBFGSpp::LBFGSBParam<float> param;
	param.epsilon = 1e-5;
	param.max_iterations = 80;
	LBFGSpp::LBFGSBSolver<float> solver(param);

	// Bounds
	uint N = pcb->objs.size()*2, M = pcb->junctions.size();
	Eigen::VectorXf lb(N+M), ub(N+M);
	for(uint i = 0; i < N; ++i) {
		float m = 1e9;
		for(uint j = 0; j < pcb->objs[i/2].size(); ++j) m = std::min(m, pcb->objs[i/2].getVertices()[2*j+(i&1)]);
		lb(i) = ((i&1) ? box.min.y : box.min.x) - m;
	}
	jcv_point wh = 3e-4*(box.max - box.min);
	for(uint i = 0; i < M; ++i) lb(N+i) = (i&1) ? box.min.y+wh.y : box.min.x+wh.x;
	for(uint i = 0; i < N; ++i) {
		float m = -1e9;
		for(uint j = 0; j < pcb->objs[i/2].size(); ++j) m = std::max(m, pcb->objs[i/2].getVertices()[2*j+(i&1)]);
		ub(i) = ((i&1) ? box.max.y : box.max.x) - m;
	}
	for(uint i = 0; i < M; ++i) ub(N+i) = (i&1) ? box.max.y-wh.y : box.max.x-wh.x;

	// start
	Eigen::VectorXf x(N+M);
	for(uint i = 0; i < N; ++i) x(i) = pcb->objs[i/2].center[i&1];
	for(uint i = 0; i < M; ++i) x(N+i) = pcb->junctions[i];

	// Minimize
	float fx;
	solver.minimize(*this, x, fx, lb, ub);

	// Update centers
	for(uint i = 0; i < N; ++i) pcb->objs[i/2].center[i&1] = x(i);
	for(uint i = 0; i < M; ++i) pcb->junctions[i] = x(N+i);
}

void CVT::getCells(std::vector<Object> &cells) const {
	float diag = jcv_norm(box.max - box.min);
	float space = diag / STEPS;

	// Create array of points
	std::vector<jcv_point> points;
	std::vector<int> from;
	for(uint i = 0; i < pcb->objs.size(); ++i) {
		uint n = pcb->objs[i].size();
		for(uint k = 0; k < n; ++k) {
			from.push_back(i);
			points.emplace_back();
			points.back().x = pcb->objs[i].center[0] + pcb->objs[i].getVertices()[2*k];
			points.back().y = pcb->objs[i].center[1] + pcb->objs[i].getVertices()[2*k+1];
		}
	}
	for(int i = 0; i < (int) pcb->junctions.size(); i += 2) {
		from.push_back(-i-1);
		points.push_back({pcb->junctions[i], pcb->junctions[i+1]});
	}
	// add edges
	for(const PCBEdge &e : pcb->edges) {
		// case 'C' and mode=1
		float r = .5*pcb->apertures[e.ap_id].parameters[0];
		jcv_point a, b;
		a.x = (e.from < 0) ? pcb->junctions[-e.from-1] : pcb->objs[e.from].center[0];
		a.y = (e.from < 0) ? pcb->junctions[-e.from] : pcb->objs[e.from].center[1];
		b.x = (e.to < 0) ? pcb->junctions[-e.to-1] : pcb->objs[e.to].center[0];
		b.y = (e.to < 0) ? pcb->junctions[-e.to] : pcb->objs[e.to].center[1];
		jcv_point v = b-a;
		float d = jcv_norm(v);
		v /= d;
		jcv_point h = {-r*v.y, r*v.x};
		int n = (d - space) / space;
		if(n&1) ++n;
		float sp = d / (n+1);
		for(int i = 1; i <= n; ++i) {
			points.push_back(a + i*sp*v + h);
			from.push_back(2*i <= n ? e.from : e.to);
			points.push_back(a + i*sp*v - h);
			from.push_back(2*i <= n ? e.from : e.to);
		}
	}
	int N = points.size();

	// Compute diagram
	jcv_diagram diagram;
	memset(&diagram, 0, sizeof(jcv_diagram));
	jcv_diagram_generate(N, points.data(), &box, nullptr, &diagram);
	const jcv_site* sites = jcv_diagram_get_sites(&diagram);

	// add cells
	cells.clear();
	std::default_random_engine re;
	std::uniform_real_distribution<float> unif(0., 1.);
	std::vector<std::vector<float>> vertices(pcb->objs.size() + pcb->junctions.size()/2);
	std::vector<std::vector<uint>> indices(pcb->objs.size() + pcb->junctions.size()/2);
	for(int i0 = 0; i0 < diagram.numsites; ++i0) {
		const jcv_site *site = &sites[i0];
		const jcv_graphedge *e = site->edges;
		const int i = from[site->index] < 0 ? pcb->objs.size()+(-from[site->index]-1)/2 : from[site->index];
		vertices[i].clear();
		indices[i].clear();
		uint s = vertices[i].size()/2;
		vertices[i].push_back(site->p.x);
		vertices[i].push_back(site->p.y);
		float col[3] = {pcb->objs[i].color[0]*.5f+.2f*unif(re), pcb->objs[i].color[1]*.5f+.2f*unif(re), pcb->objs[i].color[2]*.5f+.2f*unif(re)};
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
		cells.emplace_back(vertices[i], indices[i], col);
	}
	// for(uint i = 0; i < pcb->objs.size(); ++i) {
	// 	float col[3] = {pcb->objs[i].color[0]*.7f, pcb->objs[i].color[1]*.7f, pcb->objs[i].color[2]*.7f};
	// 	cells.emplace_back(vertices[i], indices[i], col);
	// }
	// for(uint i = pcb->objs.size(); i < vertices.size(); ++i) {
	// 	float c = .1+.1*unif(re);
	// 	float col[3] = {c+.05f*unif(re), c+.05f*unif(re), c+.05f*unif(re)};
	// 	cells.emplace_back(vertices[i], indices[i], col);
	// }

	// free
	jcv_diagram_free(&diagram);
}