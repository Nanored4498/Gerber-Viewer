#pragma once

#include <Eigen/Core>

#include "jc_voronoi.h"

#include "object.h"

class CVT {
public:
	CVT(PCB *p_pcb, const jcv_rect &p_box):
		pcb(p_pcb), box(p_box) {}

	float operator()(const Eigen::VectorXf &x, Eigen::VectorXf &grad);

	void getCells(std::vector<Object> &cells) const;

	void solve();

private:
	PCB *pcb;
	jcv_rect box;
};