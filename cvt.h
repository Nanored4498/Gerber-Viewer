#pragma once

#include <Eigen/Core>

#include "jc_voronoi.h"

#include "object.h"

class CVT {
public:
	CVT(std::vector<Object> *p_objs, const jcv_rect &p_box):
		objs(p_objs), box(p_box) {}

	float operator()(const Eigen::VectorXf &x, Eigen::VectorXf &grad);

	void getCells(std::vector<Object> &cells) const;

	void solve();

private:
	std::vector<Object> *objs;
	jcv_rect box;
};