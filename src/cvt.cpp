#include "cvt.h"

#include <iostream>
#include <random>
#include "LBFGS.h"
#include "triangulate.h"

namespace boost::polygon {
	template<> struct geometry_concept<Vec2> { typedef point_concept type; };
	template<> struct point_traits<Vec2> {
		typedef double coordinate_type;
		static inline coordinate_type get(const Vec2& v, orientation_2d orient) {
			return (orient == HORIZONTAL) ? v.x : v.y;
		}
	};
	template<> struct geometry_concept<Segment> { typedef segment_concept type; };
	template<> struct segment_traits<Segment> {
		typedef double coordinate_type;
		typedef Vec2 point_type;
		static inline point_type get(const Segment& segment, direction_1d dir) {
			return dir.to_int() ? segment.second : segment.first;
		}
	};
}

bool intersect(const Vec2 &a, const Vec2 &b, const Vec2 &u, const Vec2 &u2, double &t) {
	const Vec2 v = b - a;
   	const Vec2 w = u2 - u;
	double s = (u.x - a.x) * w.y - (u.y - a.y) * w.x;
	s /= v.x * w.y - v.y * w.x;
	if(s >= 0. && s <= t) {
		double r;
		if(std::abs(w.x) > std::abs(w.y)) r = (a.x + s*v.x - u.x) / w.x;
		else r = (a.y + s*v.y - u.y) / w.y;
		if(r >= 0. && r <= 1.) {
			t = s;
			return true;
		}
	}
	return false;
}

inline void addCurvedEdge(uint ind, const VD::edge_type *e, const std::vector<Segment> &segments, ClipperLib::Path &p, int ni) {
    if(e != nullptr && e->is_curved() && e->twin()->cell()->source_index() >= 4) {
        Vec2 a, b, c;
        uint ind2 = e->twin()->cell()->source_index()-4;
        if(e->cell()->contains_point()) {
            a = segments[ind2].first;
            b = segments[ind2].second;
            if(e->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) c = segments[ind].second;
            else c = segments[ind].first;
        } else {
            a = segments[ind].first;
            b = segments[ind].second;
            if(e->twin()->cell()->source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_END_POINT) c = segments[ind2].second;
            else c = segments[ind2].first;
        }
        b -= a;
        b /= std::sqrt(b.x*b.x + b.y*b.y);
        Vec2 b2(-b.y, b.x);
        c -= a;
        c = Vec2(c.x*b.x + c.y*b.y, c.x*b2.x + c.y*b2.y);
        double x0 = (e->vertex1()->x() - a.x)*b.x + (e->vertex1()->y() - a.y)*b.y;
        double x1 = (e->vertex0()->x() - a.x)*b.x + (e->vertex0()->y() - a.y)*b.y;
		ClipperLib::IntPoint start(std::round(e->vertex1()->x()), std::round(e->vertex1()->y()));
		ClipperLib::IntPoint end(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
        x1 -= x0;
        for(int i = 1; i < ni; ++i) {
            double x = x0 + (x1 * i) / ni;
            double dx = x - c.x;
            double y = .5 * (c.y + dx*dx/c.y);
			ClipperLib::IntPoint add(std::round(a.x + x*b.x + y*b2.x), std::round(a.y + x*b.y + y*b2.y));
			if(std::abs(add.X - start.X) + std::abs(add.Y - start.Y) <= 3) continue;
			if(std::abs(add.X - end.X) + std::abs(add.Y - end.Y) <= 3) continue;
			if(p.empty() || std::abs(add.X - p.back().X) + std::abs(add.Y - p.back().Y) > 3) p.push_back(add);
        }
    }
}

const Vec2 CVT::points[4] = {
	Vec2(-INT32_MAX, -INT32_MAX),
	Vec2(-INT32_MAX, INT32_MAX),
	Vec2(INT32_MAX, -INT32_MAX),
	Vec2(INT32_MAX, INT32_MAX)
};


CVT::CVT(PCB *p_pcb, std::vector<std::vector<float>> &&p_boundary):
	pcb(p_pcb), boundary(p_boundary) {

	assert(!boundary.empty());
	tot_area = 0.;
	border.resize(boundary.size());
	border[0].resize(boundary[0].size() / 2);
	for(uint i = 0; i < boundary[0].size(); i += 2) {
		border_box.update(boundary[0][i], boundary[0][i+1]);
		uint j = (i+2) % boundary[0].size();
		tot_area += (boundary[0][j] - boundary[0][i]) * (boundary[0][i+1] + boundary[0][j+1]);
	}
	tot_area = std::abs(tot_area);
	for(uint k = 1; k < boundary.size(); ++k) {
		double hole_area = 0.;
		border[k].resize(boundary[k].size() / 2);
		for(uint i = 0; i < boundary[k].size(); i += 2) {
			uint j = (i+2) % boundary[k].size();
			hole_area += (boundary[k][j] - boundary[k][i]) * (boundary[k][i+1] + boundary[k][j+1]);
		}
		tot_area -= std::abs(hole_area);
	}

}

void CVT::construct_voro(const Eigen::VectorXd &x, VD *vd) {
	// Create a vector of Vec2 per object
	std::vector<std::vector<Vec2>> objs(pcb->objs.size());
	Box box = border_box;
	for(uint i = 0; i < objs.size(); ++i) {
		objs[i].reserve(pcb->objs[i].size());
		for(uint j = 0; j < pcb->objs[i].size(); ++j) {
			objs[i].emplace_back(x(2*i) + pcb->objs[i].getX(j), x(2*i+1) + pcb->objs[i].getY(j));
			box.update(objs[i].back().x, objs[i].back().y);
		}
	}
	mid = (box.min() + box.max()) / 2.;
	scale = INT32_MAX / (2.5 * std::max(box.W(), box.H()));
	segments.clear();
	crs.clear();

	// Compute edge segments
	// for(const PCBEdge &e : pcb->edges) {
	// 	uint i0 = e.from < 0 ? 2*objs.size()-e.from-1 : 2*e.from;
	// 	uint i1 = e.to < 0 ? 2*objs.size()-e.to-1 : 2*e.to;
	// 	Vec2 a(x(i0), x(i0+1)), b(x(i1), x(i1+1));
	// 	double u = 0., v = 1.;
	// 	if(e.from >= 0) {
	// 		double t = 1.;
	// 		uint j = 0;
	// 		for(uint k = 0; k < objs[e.from].size(); ++k) {
	// 			uint l = (k+1) % objs[e.from].size();
	// 			if(intersect(b, a, objs[e.from][k], objs[e.from][l], t)) j = k;
	// 		}
	// 		a = b + t * (a - b);
	// 		objs[e.from].insert(objs[e.from].begin() + j, a);
	// 		u = 1. - t;
	// 	}
	// 	if(e.to >= 0) {
	// 		double t = 1.;
	// 		uint j = 0;
	// 		for(uint k = 0; k < objs[e.to].size(); ++k) {
	// 			uint l = (k+1) % objs[e.to].size();
	// 			if(intersect(a, b, objs[e.to][k], objs[e.to][l], t)) j = k;
	// 		}
	// 		b = a + t * (b - a);
	// 		objs[e.to].insert(objs[e.to].begin() + j, b);
	// 		v = t + (1. - t) * u;
	// 	}
	// 	segments.emplace_back(scale * (a - mid), scale * (b - mid));
	// 	crs.emplace_back(i0, i1, u, v);
	// }

	// Compute object segments
	for(uint i = 0; i < objs.size(); ++i) {
		std::cerr << objs[i].size() << std::endl;
		// if(objs[i].size() != 26) continue;
		for(Vec2 &v : objs[i]) v = scale * (v - mid);
		for(uint j = 0; j < objs[i].size(); ++j) {
			uint k = (j+1) % objs[i].size();
			segments.emplace_back(objs[i][j], objs[i][k]);
			crs.emplace_back(2*i, 2*i, 0., 0.);
		}
	}

	boost::polygon::construct_voronoi(points, points+4, segments.begin(), segments.end(), vd);

	// Compute border
	for(uint k = 0; k < border.size(); ++k) {
		for(uint i = 0; i < border[k].size(); ++i) {
			border[k][i].X = scale * (boundary[k][2*i] - mid.x);
			border[k][i].Y = scale * (boundary[k][2*i+1] - mid.y);
		}
	}
}

double CVT::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
	double f = 0;
	grad.setZero();

	VD vd;
	construct_voro(x, &vd);

		/*
		// energy orthogonal ditance
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
		*/

	vd.clear();
	if(f < prevF) {
		prevX = x;
		prevF = f;
	}
	std::cerr << f << std::endl;
	return f;
}

void CVT::solve() {
	LBFGSpp::LBFGSParam<double> param;
	param.epsilon = 1e-5;
	param.max_iterations = 100;
	param.m = 9;
	param.max_linesearch = 7;
	LBFGSpp::LBFGSSolver<double> solver(param);

	// start
	uint N = pcb->objs.size()*2, M = pcb->junctions.size();
	Eigen::VectorXd x(N+M);
	for(uint i = 0; i < N; ++i) x(i) = pcb->objs[i/2].center[i&1];
	for(uint i = 0; i < M; ++i) x(N+i) = pcb->junctions[i];
	prevX = x;
	prevF = std::numeric_limits<double>::max();

	// Minimize
	double fx;
	solver.minimize(*this, x, fx);

	// Update centers
	for(uint i = 0; i < N; ++i) pcb->objs[i/2].center[i&1] = prevX(i);
	for(uint i = 0; i < M; ++i) pcb->junctions[i] = prevX(N+i);
}

void CVT::getCells(std::vector<Object> &cells) {
	uint N = pcb->objs.size()*2, M = pcb->junctions.size();
	Eigen::VectorXd x(N+M);
	for(uint i = 0; i < N; ++i) x(i) = pcb->objs[i/2].center[i&1];
	for(uint i = 0; i < M; ++i) x(N+i) = pcb->junctions[i];
	VD vd;
	construct_voro(x, &vd);

	// add cells
	cells.clear();
	std::default_random_engine re;
	std::uniform_real_distribution<float> unif(0., 1.);
	std::vector<std::vector<float>> vertices(pcb->objs.size() + 1);
	std::vector<std::vector<uint>> indices(pcb->objs.size() + 1);
	for(const VD::cell_type &vd_c : vd.cells()) {
		if(vd_c.is_degenerate()) continue;
		if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) continue; // it is one of the four points added
		uint ind = vd_c.source_index()-4;
		std::cerr << "HERE" << std::endl;

		// Create polygon cell
		ClipperLib::Paths ps(1);
		ClipperLib::Path &p = ps[0];
		const VD::edge_type *e = vd_c.incident_edge();
		do {
			addCurvedEdge(ind, e, segments, p, 4);
			p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
			std::cerr << p.back().X << " " << p.back().Y << std::endl;
			e = e->prev();
		} while(e != vd_c.incident_edge());
		std::cerr << "HERE2" << std::endl;
	
		// We clip with the border
		ClipperLib::Paths clipped;
		ClipperLib::Clipper cl;
		for(const ClipperLib::IntPoint &v : p) std::cerr << v.X << " " << v.Y << std::endl;
		cl.AddPaths(ps, ClipperLib::ptSubject, true);
		cl.AddPaths(border, ClipperLib::ptClip, true);
		cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
		std::cerr << "HERE3" << std::endl;

		float col[3];
		if(crs[ind].i == crs[ind].j) {
			ind = crs[ind].i / 2;
			for(int i = 0; i < 3; ++i) col[i] = pcb->objs[ind].color[i] * .55f + .15f * unif(re);
		} else {
			ind = vertices.size()-1;
			float c = 0.1f + 0.1f * unif(re);
			for(int i = 0; i < 3; ++i) col[i] = c + 0.05f * unif(re);
		}
		std::cerr << "HERE4" << std::endl;
		for(const ClipperLib::Path &P : clipped) {
			vertices[ind].clear();
			std::vector<long long> vll;
			for(const ClipperLib::IntPoint &p : P) {
				vertices[ind].push_back(p.X / scale  + mid.x);
				vertices[ind].push_back(p.Y / scale  + mid.y);
				vll.push_back(p.X);
				std::cerr << vll.back();
				vll.push_back(p.Y);
				std::cerr << " " << vll.back() << std::endl;
			}
			std::cerr << "Tri" << std::endl;
			triangulate(vll, indices[ind]);
			std::cerr << "Tri2" << std::endl;
			cells.emplace_back(vertices[ind], indices[ind], col);
		}
	}
}