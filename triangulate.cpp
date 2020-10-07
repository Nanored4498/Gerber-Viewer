#include <algorithm>
#include <vector>
#include <map>

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
	const auto prev = [N, start](uint j) { return start + (j - start + N-1) % N; };
	const auto next = [N, start](uint j) { return start + (j - start + 1) % N; };
	const auto compY = [&](const uint i, const uint j)->bool {
		float yi = vertices[3*i+1], yj = vertices[3*j+1];
		return yi < yj || (yi == yj && vertices[3*i] < vertices[3*j]);
	};
	vector<uint> order(N);
	Edge *e0 = new Edge(start, next(start)), *e = e0;
	for(uint i = 0; i < N; ++i) {
		order[i] = start + i;
		if(i > 0) {
			e->next = new Edge(order[i], next(order[i]));
			e->next->prev = e;
			e = e->next;
		}
	}
	e->next = e0;
	e0->prev = e;
	sort(order.begin(), order.end(), compY);
	bool trigo = turnLeft(vertices, prev(order[0]), order[0], next(order[0]));

	typedef pair<uint, uint> puu;
	const auto compE = [&](const puu &a, const puu &b)->bool {
		float xa0 = vertices[3*a.first], xb0 = vertices[3*b.first];
		float ya0 = vertices[3*a.first+1], yb0 = vertices[3*b.first+1];
		if(ya0 < yb0) return xa0 + (yb0 - ya0) * (vertices[3*a.second] - xa0) / (vertices[3*a.second+1] - ya0) < xb0;
		else return xa0 < xb0 + (ya0 - yb0) * (vertices[3*b.second] - xb0) / (vertices[3*b.second+1] - yb0);
	};
	map<puu, uint, decltype(compE)> BST(compE);
	vector<bool> loc_max(order.size(), false); 
	vector<puu> diagonals;
	for(uint j : order) {
		uint i = prev(j), k = next(j);
		if(compY(j, i)) {
			if(compY(j, k)) {
				if(turnLeft(vertices, i, j, k) == trigo) { // start vertex
					BST[{i, j}] = j;
				} else { // split vertex
					auto left = -- BST.upper_bound({j, j});
					uint v = left->second;
				}
			}
		}
	}
}