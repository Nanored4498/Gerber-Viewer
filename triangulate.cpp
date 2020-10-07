#include <algorithm>
#include <vector>
#include <map>
#include <iostream>

using namespace std;

bool turnLeft(const vector<float> &vertices, uint i, uint j, uint k) {
	float jx = vertices[3*j], jy = vertices[3*j+1];
	float ax = jx - vertices[3*i], ay = jy - vertices[3*i+1];
	float bx = vertices[3*i] - jx, by = vertices[3*i+1] - jy;
	return -ay * bx + ax * by > 0;
}

struct Edge {
	uint a, b;
	Edge *prev, *next;
	Edge(uint a, uint b): a(a), b(b) {}
};

void triangulate(const vector<float> &vertices, vector<uint> &indices, uint start) {
	size_t N = vertices.size()/3 - start;
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
			in[i]->next = out[i];
			out[i]->prev = in[i];
		}
	}
	in[0] = out[N-1];
	in[0]->next = out[0];
	out[0]->prev = in[0];
	sort(order.begin(), order.end(), compY);
	if(!turnLeft(vertices, in[0]->a, order[0], out[0]->b)) { // the contour is clockwise
		// we make it counter-clockwise
		for(uint i = 0; i < N; ++i) {
			swap(out[i]->a, out[i]->b);
			swap(out[i], in[i]);
		}
	}

	typedef pair<uint, uint> puu;
	const auto compE = [&](const Edge *a, const Edge *b)->bool {
		float xa0 = vertices[3*a->a], xb0 = vertices[3*b->a];
		float ya0 = vertices[3*a->a+1], yb0 = vertices[3*b->a+1];
		if(ya0 < yb0) return xa0 + (yb0 - ya0) * (vertices[3*a->b] - xa0) / (vertices[3*a->b+1] - ya0) < xb0;
		else return xa0 < xb0 + (ya0 - yb0) * (vertices[3*b->b] - xb0) / (vertices[3*b->b+1] - yb0);
	};
	const auto isMerge = [&](uint j)->bool {
		uint i = in[j-start]->a, k = out[j-start]->b;
		return compY(i, j) && compY(k, j) && !turnLeft(vertices, i, j, k);
	};
	const auto addEdge = [&](uint j, uint v)->void {
		Edge *new1 = new Edge(j, v), *new2 = new Edge(v, j);
		new1->next = out[v-start];
		new1->prev = in[j-start];
		new2->next = out[j-start];
		new2->prev = in[v-start];
		cerr << "add " << j << " " << v << endl;
	};
	map<Edge*, uint, decltype(compE)> BST(compE);
	vector<bool> loc_max(order.size(), false); 
	vector<puu> diagonals;
	for(uint j : order) {
		uint i = in[j-start]->a, k = out[j-start]->b;
		if(compY(j, i)) {
			if(compY(j, k)) {
				if(turnLeft(vertices, i, j, k)) { // start vertex
					BST[out[j-start]] = j;
				} else { // split vertex
					auto right = BST.upper_bound(new Edge(j, j));
					if(right == BST.end()) cerr << "temp BUG" << endl;
					else {
						uint v = right->second;
						addEdge(j, v);
					}
					BST[out[j-start]] = j;
				}
			} else { // left side vertex
				auto right = BST.upper_bound(new Edge(j, j));
				if(right == BST.end()) cerr << "temp BUG" << endl;
				else {
					uint v = right->second;
					if(isMerge(v)) addEdge(j, v);
					right->second = j;
				}
			}
		} else {
			if(compY(k, j)) {
				if(turnLeft(vertices, i, j, k)) { // end vertex
					uint v = BST[in[j-start]];
					if(isMerge(v)) addEdge(j, v);
					BST.erase(in[j-start]);
				} else { // merge vertex
				}
			} else { // right side vertex
			}
		}
	}
}