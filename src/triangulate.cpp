#include "triangulate.h"

#include <algorithm>
#include <map>

using namespace std;
typedef long long ll;
#define X(i) coords[(i)<<1]
#define Y(i) coords[((i)<<1) + 1]

bool turnLeft(const vector<ll> &coords, uint i, uint j, uint k) {
	ll jx = X(j), jy = Y(j);
	ll ax = jx - X(i), ay = jy - Y(i);
	ll bx = X(k) - jx, by = Y(k) - jy;
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

void triangulate(const vector<ll> &coords, vector<uint> &indices) {
	indices.clear();
	size_t N = coords.size()/2;
	const auto compY = [&](const uint i, const uint j)->bool {
		ll yi = Y(i), yj = Y(j);
		return yi < yj || (yi == yj && X(i) < X(j));
	};
	vector<uint> order(N);
	vector<Edge*> in(N), out(N);
	for(uint i = 0; i < N; ++i) {
		order[i] = i;
		out[i] = new Edge(i, (i+1)%N);
		if(i > 0) {
			in[i] = out[i-1];
			out[i]->connect(in[i]);
		}
	}
	in[0] = out[N-1];
	out[0]->connect(in[0]);
	sort(order.begin(), order.end(), compY);
	if(!turnLeft(coords, in[order[0]]->a, order[0], out[order[0]]->b)) { // the contour is clockwise
		// we make it counter-clockwise
		for(uint i = 0; i < N; ++i) {
			swap(out[i]->a, out[i]->b);
			swap(out[i]->prev, out[i]->next);
			swap(out[i], in[i]);
		}
	}

	const auto compE = [&](const Edge *e1, const Edge *e2)->bool {
		uint a1 = e1->a, b1 = e1->b, a2 = e2->a, b2 = e2->b;
		if(Y(a1) > Y(b1)) swap(a1, b1);
		if(Y(a2) > Y(b2)) swap(a2, b2);
		ll xa1 = X(a1), xa2 = X(a2);
		ll ya1 = Y(a1), ya2 = Y(a2);
		if(ya1 < ya2) xa1 += (ya2 - ya1) * (X(b1) - xa1) / (Y(b1) - ya1);
		else if(ya1 > ya2) xa2 += (ya1 - ya2) * (X(b2) - xa2) / (Y(b2) - ya2);
		if(xa1 != xa2) return xa1 < xa2;
		else return e1 < e2;
	};
	const auto isMerge = [&](uint j)->bool {
		uint i = in[j]->a, k = out[j]->b;
		return compY(i, j) && compY(k, j) && !turnLeft(coords, i, j, k);
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
		for(Edge *e : {e1, e2}) if(e->next->next->next == e) {
			indices.push_back(e->a);
			indices.push_back(e->b);
			indices.push_back(e->next->b);
		}
	};
	map<Edge*, Edge*, decltype(compE)> BST(compE);
	const auto right_edge = [&](uint j) {
		Edge *e = new Edge(j, j);
		auto right = BST.upper_bound(e);
		delete e;
		return right;
	};
	for(uint j : order) {
		Edge *e_in = in[j], *e_out = out[j];
		uint i = e_in->a, k = e_out->b;
		if(compY(j, i)) {
			if(compY(j, k)) {
				if(turnLeft(coords, i, j, k)) { // start vertex
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
			if(compY(k, j)) {
				if(turnLeft(coords, i, j, k)) { // end vertex
					Edge *helper = BST[e_in];
					if(isMerge(helper->b)) addEdge(e_in, helper);
					BST.erase(e_in);
				} else { // merge vertex
					Edge *helper = BST[e_in], *in2 = e_in;
					if(isMerge(helper->b)) {
						addEdge(e_in, helper);
						in2 = e_out->prev;
					}
					BST.erase(e_in);
					auto right = right_edge(j);
					if(isMerge(right->second->b)) addEdge(e_out->prev, right->second);
					right->second = in2;
				}
			} else { // right side vertex
				Edge *helper = BST[e_in];
				if(isMerge(helper->b)) addEdge(e_in, helper);
				BST.erase(e_in);
				BST[e_out] = e_out->prev;
			}
		}
	}
	
	uint Es = out.size();
	for(uint i = 0; i < Es; ++i) {
		if(out[i]->next == nullptr) continue;
		Edge *e = out[i], *ey0 = e, *ey1 = e, *e2 = e->next;
		while(e2 != e) {
			if(compY(e2->b, ey0->b)) ey0 = e2; 
			if(compY(ey1->b, e2->b)) ey1 = e2; 
			e2 = e2->next;
		}
		vector<Edge*> order = {ey0};
		e = ey0->prev;
		e2 = ey0->next;
		while(e != ey1 || e2 != ey1) {
			if(compY(e->b, e2->b)) {
				order.push_back(e);
				e = e->prev;
			} else {
				order.push_back(e2);
				e2 = e2->next;
			}
		}
		order.push_back(ey1);
		uint n = order.size();

		vector<Edge*> stack = {order[0], order[1]};
		for(uint i = 2; i+1 < n; ++i) {
			e = order[i];
			Edge *e2 = stack.back();
			if(compY(e->a, e->b)) { // e is right side
				if(e->a == e2->b) { // e2 is in the same side
					do {
						stack.pop_back();
						if(stack.empty()) break;
						Edge *e3 = stack.back();
						if(turnLeft(coords, e3->b, e2->b, e->b)) {
							addEdge(e, e3);
							e = e3->next;
							e2 = e3;
						} else break;
					} while(!stack.empty());
					stack.push_back(e->prev);
					stack.push_back(e);
				} else { // e2 is in the opposite side
					Edge *e3 = e->next;
					do {
						stack.pop_back();
						addEdge(e, e2);
						e2 = stack.back();
					} while(stack.size() > 1);
					stack.pop_back();
					e = e3->prev;
					stack.push_back(e->prev);
					stack.push_back(e);
				}
			} else { // Left side
				if(e->b == e2->a) { // e2 is in the same side
					do {
						stack.pop_back();
						if(stack.empty()) break;
						Edge *e3 = stack.back();
						if(turnLeft(coords, e->b, e2->b, e3->b)) {
							addEdge(e, e3);
							e2 = e3;
						} else break;
					} while(!stack.empty());
					stack.push_back(e->next);
					stack.push_back(e);
				} else { // e2 is in the opposite side
					Edge *e3 = e;
					do {
						stack.pop_back();
						addEdge(e3, e2);
						e3 = e2->next;
						e2 = stack.back();
					} while(stack.size() > 1);
					stack.pop_back();
					stack.push_back(e->next);
					stack.push_back(e);
				}
			}
		}
		e = order.back();
		stack.pop_back();
		while(!stack.empty()) {
			Edge *e2 = stack.back();
			stack.pop_back();
			if(!stack.empty()) {
				addEdge(e, e2);
				if(compY(e2->a, e2->b)) { // e2 is right side
					e = e2->next;
				}
			}
		}
		
		for(Edge *e : order) e->next = nullptr;
	}

	for(Edge *e : out) delete e;
}