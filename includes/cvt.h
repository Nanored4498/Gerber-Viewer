#pragma once

#include <boost/polygon/voronoi.hpp>
#include <Eigen/Core>
#include "clipper.hpp"
#include "object.h"
#include "vec2.h"

typedef std::pair<Vec2, Vec2> Segment;
typedef boost::polygon::voronoi_diagram<double> VD;

struct Box {
	double x0 = std::numeric_limits<double>::max();
	double x1 = std::numeric_limits<double>::lowest();
	double y0 = std::numeric_limits<double>::max();
	double y1 = std::numeric_limits<double>::lowest();

	template <typename Scalar>
	inline void update(Scalar x, Scalar y) {
		x0 = std::min(x0, (double) x);
		x1 = std::max(x1, (double) x);
		y0 = std::min(y0, (double) y);
		y1 = std::max(y1, (double) y);
	}
	inline double W() const { return x1-x0; } 
	inline double H() const { return y1-y0; }
	inline Vec2 min() const { return {x0, y0}; }
	inline Vec2 max() const { return {x1, y1}; }
};

struct self_intersection_error : public std::exception {
	const std::string m_msg;
	self_intersection_error(const char* msg):
		m_msg("An edge of the path intersects an other edge of the path (detected by: " + std::string(msg) + ")")
		{}
	const char* what() const throw() { return m_msg.c_str(); }
};

struct ChainRule {
	uint i, j;
	double u, v;
	ChainRule(uint i, uint j, double u, double v): i(i), j(j), u(u), v(v) {}
};

class CVT {
public:
	CVT(PCB *p_pcb, std::vector<std::vector<float>> &&p_boundary);

	double operator()(const Eigen::VectorXd &x, Eigen::VectorXd &grad);

	void getCells(std::vector<Object> &cells);

	void solve();

private:
	PCB *pcb;
	std::vector<std::vector<float>> boundary;
	ClipperLib::Paths border;
    Box border_box;
	double tot_area;
    double scale;
	Vec2 mid;
    static const Vec2 points[4];
    std::vector<Segment> segments;
	std::vector<ChainRule> crs;

	Eigen::VectorXd prevX;
	double prevF;

	double foundError(const Eigen::VectorXd &x, Eigen::VectorXd &grad, const char* msg);

    void construct_voro(const Eigen::VectorXd &x, VD *vd);
};