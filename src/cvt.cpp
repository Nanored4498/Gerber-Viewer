#include "cvt.h"

#include <iostream>
#include <random>
#include "LBFGS.h"
#include "triangulate.h"

const bool printLineSearchError = true;
const bool bounded = true;

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

inline void getMomentsSeg(const Vec2 &a, const Vec2 &b, double &M0, Vec2 &M1, Vec2 &M2) {
	Vec2 d(a.y - b.y, b.x - a.x);
	Vec2 s = a+b;
	M0 += d.x * s.x / 2.;
	M1 += d * (a*s + b*b) / 6.;
	M2 += d * (a*a + b*b) * s / 12.;
}

inline void getYMoments2(const Vec2 &a, const Vec2 &b, double l, double &M1a, double &M1b, double &M2) {
	double dx = b.x - a.x;
	double ta = a.x/l, tb = b.x/l;
	double bb = b.y*b.y, ba = b.y*a.y, aa = a.y*a.y;
	double m1b = dx * ((3.*tb+ta)*bb + 2.*(ta+tb)*ba + (3.*ta+tb)*aa) / 24.;
	M1b += m1b;
	M1a += dx * (bb + ba + aa) / 6. - m1b;
	M2 += dx * (b.y+a.y) * (bb+aa) / 12.;
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

bool insideLoop(const std::vector<float> &loop, double x, double y) {
	bool inside = false;
	for (uint i = 0; i < loop.size(); i += 2) {
		uint j = (i+2) % loop.size();
		if(loop[i+1] == loop[j+1]) continue;
		if(loop[j+1] == y) {
			if(loop[j] > x && loop[j+1] < loop[i+1]) inside = !inside;
		} else if(loop[i+1] == y) {
			if(loop[i] > x && loop[j+1] > loop[i+1]) inside = !inside;
		} else if((loop[j+1] > y) != (loop[i+1] > y)) {
			double x2 = loop[i] + (loop[j] - loop[i]) * (y - loop[i+1]) / (loop[j+1] - loop[i+1]);
			if(x2 > x) inside = !inside;
		}
	}
	return inside;
}

bool inside(const std::vector<std::vector<float>> &boundary, double x, double y) {
	if(insideLoop(boundary[0], x, y)) {
		for(uint i = 1; i < boundary.size(); ++i)
			if(insideLoop(boundary[i], x, y))
				return false;
		return true;
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
			if(std::abs(add.X - start.X) + std::abs(add.Y - start.Y) <= 30) continue;
			if(std::abs(add.X - end.X) + std::abs(add.Y - end.Y) <= 30) continue;
			if(p.empty() || std::abs(add.X - p.back().X) + std::abs(add.Y - p.back().Y) > 30) p.push_back(add);
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
	mid = (border_box.min() + border_box.max()) / 2.;
	scale = INT32_MAX / ((bounded ? 2.5 : 7.) * std::max(border_box.W(), border_box.H()));

	// Compute border
	for(uint k = 0; k < border.size(); ++k) {
		for(uint i = 0; i < border[k].size(); ++i) {
			border[k][i].X = scale * (boundary[k][2*i] - mid.x);
			border[k][i].Y = scale * (boundary[k][2*i+1] - mid.y);
		}
	}
}

bool CVT::construct_voro(const Eigen::VectorXd &x, VD *vd) {
	// Create a vector of Vec2 per object
	std::vector<std::vector<Vec2>> objs(pcb->objs.size());
	for(uint i = 0; i < objs.size(); ++i) {
		objs[i].reserve(pcb->objs[i].size());
		for(uint j = 0; j < pcb->objs[i].size(); ++j) {
			objs[i].emplace_back(x(2*i) + pcb->objs[i].getX(j), x(2*i+1) + pcb->objs[i].getY(j));
			if(bounded && !inside(boundary, objs[i].back().x, objs[i].back().y)) return false;
		}
	}
	if(bounded)
		for(uint i = 2*objs.size(); i < x.size(); i += 2)
			if(!inside(boundary, x(i), x(i+1)))
				return false;
	segments.clear();
	crs.clear();

	// Compute edge segments
	for(const PCBEdge &e : pcb->edges) {
		uint i0 = e.from < 0 ? 2*objs.size()-e.from-1 : 2*e.from;
		uint i1 = e.to < 0 ? 2*objs.size()-e.to-1 : 2*e.to;
		Vec2 a(x(i0), x(i0+1)), b(x(i1), x(i1+1));
		double u = 0., v = 1.;
		if(e.from >= 0) {
			double t = 1.;
			uint j = 0;
			for(uint k = 0; k < objs[e.from].size(); ++k) {
				uint l = (k+1) % objs[e.from].size();
				if(intersect(b, a, objs[e.from][k], objs[e.from][l], t)) j = k;
			}
			uint k = (j+1) % objs[e.from].size();
			a = b + t * (a - b);
			if(std::abs(std::round(scale * (a.x - objs[e.from][j].x))) + std::abs(std::round(scale * (a.y - objs[e.from][j].y))) < 20) {
				a = objs[e.from][j];
			} else if(std::abs(std::round(scale * (a.x - objs[e.from][k].x))) + std::abs(std::round(scale * (a.y - objs[e.from][k].y))) < 20) {
				a = objs[e.from][k];
			} else {
				objs[e.from].insert(objs[e.from].begin() + k, a);
			}
			u = 1. - t;
		}
		if(e.to >= 0) {
			double t = 1.;
			uint j = 0;
			for(uint k = 0; k < objs[e.to].size(); ++k) {
				uint l = (k+1) % objs[e.to].size();
				if(intersect(a, b, objs[e.to][k], objs[e.to][l], t)) j = k;
			}
			uint k = (j+1) % objs[e.to].size();
			b = a + t * (b - a);
			if(std::abs(std::round(scale * (b.x - objs[e.to][j].x))) + std::abs(std::round(scale * (b.y - objs[e.to][j].y))) < 20)
				b = objs[e.to][j];
			else if(std::abs(std::round(scale * (b.x - objs[e.to][k].x))) + std::abs(std::round(scale * (b.y - objs[e.to][k].y))) < 20)
				b = objs[e.to][k];
			else
				objs[e.to].insert(objs[e.to].begin() + k, b);
			v = t + (1. - t) * u;
		}
		segments.emplace_back(scale * (a - mid), scale * (b - mid));
		crs.emplace_back(i0, i1, u, v);
	}

	// Compute object segments
	for(uint i = 0; i < objs.size(); ++i) {
		for(Vec2 &v : objs[i]) v = scale * (v - mid);
		for(uint j = 0; j < objs[i].size(); ++j) {
			uint k = (j+1) % objs[i].size();
			segments.emplace_back(objs[i][j], objs[i][k]);
			crs.emplace_back(2*i, 2*i, 0., 0.);
		}
	}

	boost::polygon::construct_voronoi(points, points+4, segments.begin(), segments.end(), vd);
	return true;
}

inline double CVT::foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg) {
	if(prevF != std::numeric_limits<double>::max()) {
		grad = x - prevX;
		double len = grad.norm();
		grad *= 100./len;
		return prevF + 100.*len;
	}
	throw self_intersection_error(msg);
}

double CVT::operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad) {
	double f = 0;
	grad.setZero();

	VD vd;
	if(!construct_voro(x, &vd)) return foundError(x, grad, "A point is outside");

	for(const VD::cell_type &vd_c : vd.cells()) {
		if(vd_c.is_degenerate()) continue;
		if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) continue; // it is one of the four points added
		uint ind = vd_c.source_index()-4;

		// Create polygon cell
		ClipperLib::Paths ps(1);
		ClipperLib::Path &p = ps[0];
		const VD::edge_type *e = vd_c.incident_edge();
		int countEloop = 0;
		do {
			if(++ countEloop > 1000) return foundError(x, grad, "infinite looping over edges");
			if(e->is_infinite()) return foundError(x, grad, "infinite edge");
			addCurvedEdge(ind, e, segments, p, 7);
			p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
			e = e->prev();
		} while(e != vd_c.incident_edge());
		ClipperLib::cInt area = (p[0].X - p.back().X) * (p[0].Y + p.back().Y);
		for(uint i = 1; i < p.size(); ++i) area += (p[i].X - p[i-1].X) * (p[i].Y + p[i-1].Y);
		if(area < 0.) return foundError(x, grad, "negative area");
	
		// We clip with the border
		ClipperLib::Paths clipped;
		ClipperLib::Clipper cl;
		try {
			cl.AddPaths(ps, ClipperLib::ptSubject, true);
			cl.AddPaths(border, ClipperLib::ptClip, true);
			cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);
		} catch(const ClipperLib::clipperException &e) {
			return foundError(x, grad, "clipper error");
		}

		Vec2 a, v;
		double l;
		bool isPoint = vd_c.contains_point();
		if(isPoint) {
			if(vd_c.source_category() == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
				l = crs[ind].u;
				a = segments[ind].first / scale + mid;
			} else {
				l = crs[ind].v;
				a = segments[ind].second / scale + mid;
			}
		} else {
			a = segments[ind].first / scale + mid;
			v = segments[ind].second / scale + mid - a;
			l = v.norm();
			v /= l;
		}

		for(const ClipperLib::Path &P0 : clipped) {
			size_t n = P0.size() + 1;
			std::vector<Vec2> P;
			P.reserve(n);
			for(const ClipperLib::IntPoint &p : P0) P.emplace_back(p.X / scale + mid.x, p.Y / scale + mid.y);
			P.push_back(P[0]);
			if(isPoint) {
				double M0 = 0.;
				Vec2 M1(0., 0.), M2(0., 0.);
				for(uint i = 1; i < n; ++i) getMomentsSeg(P[i], P[i-1], M0, M1, M2);
				Vec2 add = .5 * (M2 - 2.*M1*a + M0*a*a);
				f += add.x + add.y;
				double gx = M0*a.x - M1.x;
				double gy = M0*a.y - M1.y;
				grad(crs[ind].i) += (1. - l) * gx;
				grad(crs[ind].i+1) += (1. - l) * gy;
				grad(crs[ind].j) += l * gx;
				grad(crs[ind].j+1) += l * gy;
			} else {
				for(Vec2 &u : P) {
					u -= a;
					u = Vec2(v.x*u.x + v.y*u.y, -v.y*u.x + v.x*u.y);
				}
				double M1a = 0., M1b = 0., M2 = 0.;
				for(uint i = 1; i < n; ++i) getYMoments2(P[i], P[i-1], l, M1a, M1b, M2);
				f += .5 * M2;
				double coeffI = (1. - crs[ind].u) * M1a + (1. - crs[ind].v) * M1b;
				double coeffJ = crs[ind].u * M1a + crs[ind].v * M1b;
				grad(crs[ind].i) += coeffI * v.y;
				grad(crs[ind].i+1) -= coeffI * v.x;
				grad(crs[ind].j) += coeffJ * v.y;
				grad(crs[ind].j+1) -= coeffJ * v.x;
			}
		}
	}

	size_t N = pcb->objs.size();
	double dotCoeff = tot_area / N;
	for(uint i = 0; i < segments.size(); ++i) if(crs[i].i != crs[i].j) {
		Vec2 h;
		h.x = crs[i].i < 2*N ? pcb->objs[crs[i].i/2].center[1] : pcb->junctions[crs[i].i-2*N+1];
		h.x -= crs[i].j < 2*N ? pcb->objs[crs[i].j/2].center[1] : pcb->junctions[crs[i].j-2*N+1];
		h.y = crs[i].j < 2*N ? pcb->objs[crs[i].j/2].center[0] : pcb->junctions[crs[i].j-2*N];
		h.y -= crs[i].i < 2*N ? pcb->objs[crs[i].i/2].center[0] : pcb->junctions[crs[i].i-2*N];
		h /= h.norm();
		double d = dot(h, (segments[i].second - segments[i].first) / scale);
		f += .5 * dotCoeff * d*d;
		Vec2 g = (dotCoeff * (crs[i].v - crs[i].u) * d) * h;
		grad(crs[i].i) -= g.x;
		grad(crs[i].i+1) -= g.y;
		grad(crs[i].j) += g.x;
		grad(crs[i].j+1) += g.y;
	}

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
	param.max_linesearch = 15;
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
	try {
		solver.minimize(*this, x, fx);
	} catch(const std::runtime_error &e) {
		if(printLineSearchError) std::cerr << "[Smoother Runtime Error]: " << e.what() << std::endl;
	} catch(const std::logic_error &e) {
		std::cerr << "[Smoother Logic Error]: " << e.what() << std::endl;
	} catch(const self_intersection_error &e) {
		std::cerr << "[Smoother Self Intersection Error]: " << e.what() << std::endl;
	}

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

		// Create polygon cell
		ClipperLib::Paths ps(1);
		ClipperLib::Path &p = ps[0];
		const VD::edge_type *e = vd_c.incident_edge();
		do {
			addCurvedEdge(ind, e, segments, p, 4);
			p.emplace_back(std::round(e->vertex0()->x()), std::round(e->vertex0()->y()));
			e = e->prev();
		} while(e != vd_c.incident_edge());
	
		// We clip with the border
		ClipperLib::Paths clipped;
		ClipperLib::Clipper cl;
		cl.AddPaths(ps, ClipperLib::ptSubject, true);
		cl.AddPaths(border, ClipperLib::ptClip, true);
		cl.Execute(ClipperLib::ctIntersection, clipped, ClipperLib::pftNonZero, ClipperLib::pftNonZero);

		float col[3];
		if(crs[ind].i == crs[ind].j) {
			ind = crs[ind].i / 2;
			for(int i = 0; i < 3; ++i) col[i] = pcb->objs[ind].color[i] * .55f + .15f * unif(re);
		} else {
			ind = vertices.size()-1;
			float c = 0.1f + 0.1f * unif(re);
			for(int i = 0; i < 3; ++i) col[i] = c + 0.05f * unif(re);
		}
		for(const ClipperLib::Path &P : clipped) {
			vertices[ind].clear();
			std::vector<long long> vll;
			for(const ClipperLib::IntPoint &p : P) {
				vertices[ind].push_back(p.X / scale  + mid.x);
				vertices[ind].push_back(p.Y / scale  + mid.y);
				vll.push_back(p.X);
				vll.push_back(p.Y);
			}
			triangulate(vll, indices[ind]);
			cells.emplace_back(vertices[ind], indices[ind], col);
		}
	}

	/*
	for(const Segment &s : segments) {
		Vec2 a = s.first / scale + mid;
		Vec2 b = s.second / scale + mid;
		Vec2 v = b-a;
		v *= .06 / v.norm();
		std::swap(v.x, v.y);
		v.x = -v.x;
		std::vector<float> vert;
		vert.push_back(a.x - v.x); vert.push_back(a.y - v.y);
		vert.push_back(a.x + v.x); vert.push_back(a.y + v.y);
		vert.push_back(b.x + v.x); vert.push_back(b.y + v.y);
		vert.push_back(b.x - v.x); vert.push_back(b.y - v.y);
		std::vector<uint> inds {0, 1, 2, 0, 2, 3};
		float col[3] {0., 0., 0.};
		cells.emplace_back(vert, inds, col);
	}
	*/
}