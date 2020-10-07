#include <algorithm>
#include <vector>
#include <map>
#include <iostream>

using namespace std;

bool turnLeft(const vector<float> &vertices, uint i, uint j, uint k) {
	float jx = vertices[3*j], jy = vertices[3*j+1];
	float ax = jx - vertices[3*i], ay = jy - vertices[3*i+1];
	float bx = vertices[3*k] - jx, by = vertices[3*k+1] - jy;
	return -ay * bx + ax * by > 0;
}

struct Edge {
	uint a, b;
	Edge *prev, *next;
	Edge(uint a, uint b): a(a), b(b) {}
	void connect(Edge *e) {
		prev = e;
		e->next = this;
	}
};

void triangulate(const vector<float> &vertices, vector<uint> &indices, uint start) {
	size_t N = vertices.size()/3 - start;
	cerr << "triangulate: " << N << endl;
	const auto compY = [&](const uint i, const uint j)->bool {
		float yi = vertices[3*i+1], yj = vertices[3*j+1];
		return yi < yj || (yi == yj && vertices[3*i] < vertices[3*j]);
	};
	vector<uint> order(N);
	vector<Edge*> in(N), out(N);
	for(uint i = 0; i < N; ++i) {
		order[i] = start + i;
		out[i] = new Edge(order[i], i+1==N ? start : order[i]+1);
		if(i > 0) {
			in[i] = out[i-1];
			out[i]->connect(in[i]);
		}
	}
	in[0] = out[N-1];
	out[0]->connect(in[0]);
	sort(order.begin(), order.end(), compY);
	if(!turnLeft(vertices, in[order[0]-start]->a, order[0], out[order[0]-start]->b)) { // the contour is clockwise
		// we make it counter-clockwise
		for(uint i = 0; i < N; ++i) {
			swap(out[i]->a, out[i]->b);
			swap(out[i]->prev, out[i]->next);
			swap(out[i], in[i]);
		}
	}

	const auto compE = [&](const Edge *e1, const Edge *e2)->bool {
		float xa0 = vertices[3*e1->a], xb0 = vertices[3*e2->a];
		float ya0 = vertices[3*e1->a+1], yb0 = vertices[3*e2->a+1];
		if(ya0 < yb0) return xa0 + (yb0 - ya0) * (vertices[3*e1->b] - xa0) / (vertices[3*e1->b+1] - ya0) < xb0;
		else return xa0 < xb0 + (ya0 - yb0) * (vertices[3*e2->b] - xb0) / (vertices[3*e2->b+1] - yb0);
	};
	const auto isMerge = [&](uint j)->bool {
		uint i = in[j-start]->a, k = out[j-start]->b;
		return compY(i, j) && compY(k, j) && !turnLeft(vertices, i, j, k);
	};
	const auto addEdge = [&](Edge *e1, Edge *e2)->void {
		uint i = e1->b, j = e2->b;
		Edge *new1 = new Edge(i, j), *new2 = new Edge(j, i);
		e1->next->connect(new2);
		e2->next->connect(new1);
		new1->connect(e1);
		new2->connect(e2);
		out.push_back(new1);
		out.push_back(new2);
		cerr << "add " << i << " " << j << endl;
	};
	map<Edge*, Edge*, decltype(compE)> BST(compE);
	const auto right_edge = [&](uint j) {
		Edge *e = new Edge(j, j);
		auto right = BST.upper_bound(e);
		delete e;
		if(right == BST.end()) {
			cerr << "temp BUG" << endl;
			cerr << BST.size() << endl;
		}
		return right;
	};
	for(uint j : order) {
		Edge *e_in = in[j-start], *e_out = out[j-start];
		uint i = e_in->a, k = e_out->b;
		if(compY(j, i)) {
			if(compY(j, k)) {
				if(turnLeft(vertices, i, j, k)) { // start vertex
					BST[e_out] = e_in;
				} else { // split vertex
					auto right = right_edge(j);
					addEdge(e_in, right->second);
					right->second = e_in;
					BST[e_out] = e_out->prev;
				}
			} else { // left side vertex
				auto right = right_edge(j);
				if(isMerge(right->second->b)) addEdge(e_in, right->second);
				right->second = e_in;
			}
		} else {
			if(compY(k, j)) { // end or merge vertex
				if(turnLeft(vertices, i, j, k)) { // end vertex
					Edge *helper = BST[e_in];
					if(isMerge(helper->b)) addEdge(e_in, helper);
					BST.erase(e_in);
				} else { // merge vertex
					Edge *helper = BST[e_in];
					if(isMerge(helper->b)) addEdge(e_in, helper);
					BST.erase(e_in);
					auto right = right_edge(j);
					if(isMerge(right->second->b)) addEdge(e_out->prev, right->second);
					right->second = e_out->prev;
				}
			} else { // right side vertex
				Edge *helper = BST[e_in];
				if(isMerge(helper->b)) addEdge(e_in, helper);
				BST.erase(e_in);
				BST[e_out] = e_out->prev;
			}
		}
	}

	// TODO: Replace this code for convex polygons by a code for y-monotone polygons 
	for(uint i = 0; i < out.size(); ++i) {
		if(out[i]->next == nullptr) continue;
		Edge *e = out[i];
		uint v = e->prev->a;
		while(e != nullptr) {
			if(e->a != v && e->b != v) {
				indices.push_back(v);
				indices.push_back(e->a);
				indices.push_back(e->b);
			}
			e->prev->next = nullptr;
			e = e->next;
		}
	}

	for(Edge *e : out) delete e;
}