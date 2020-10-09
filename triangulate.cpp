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
	// cerr << "triangulate: " << N << endl;
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
		for(Edge *e : {e1, e2}) if(e->next->next->next == e) {
			indices.push_back(e->a);
			indices.push_back(e->b);
			indices.push_back(e->next->b);
		}
		// cerr << "add " << i << " " << j << endl;
	};
	map<Edge*, Edge*, decltype(compE)> BST(compE);
	const auto right_edge = [&](uint j) {
		Edge *e = new Edge(j, j);
		auto right = BST.upper_bound(e);
		delete e;
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

	uint Es = out.size();
	for(uint i = 0; i < Es; ++i) {
		if(out[i]->next == nullptr) continue;
		Edge *e = out[i];
		while(compY(e->a, e->b) || compY(e->next->b, e->b)) e = e->next;
		Edge *el = e->next, *er = e->prev;
		vector<Edge*> order = {e};
		while(compY(el->a, el->b) && compY(er->b, er->a)) {
			if(compY(el->b, er->b)) {
				order.push_back(el);
				el = el->next;
			} else {
				order.push_back(er);
				er = er->prev;
			}
		}
		while(compY(el->a, el->b)) {
			order.push_back(el);
			el = el->next;
		}
		while(compY(er->b, er->a)) {
			order.push_back(er);
			er = er->prev;
		}
		uint n = order.size();
		// cerr << "monotone: " << n << endl;
		// for(Edge* e : order) cerr << e->b << " "; cerr << endl;

		vector<Edge*> stack = {order[0], order[1]};
		for(uint i = 2; i+1 < n; ++i) {
			e = order[i];
			Edge *e2 = stack.back();
			if(compY(e->a, e->b)) { // e is right side
				if(e->a == e2->b) { // e2 is in the same side
					// cerr << e->b << " " << e2->b << " right same" << endl;
					do {
						stack.pop_back();
						if(stack.empty()) break;
						Edge *e3 = stack.back();
						if(turnLeft(vertices, e3->b, e2->b, e->b)) {
							addEdge(e, e3);
							e = e3->next;
							e2 = e3;
						} else break;
					} while(!stack.empty());
					stack.push_back(e->prev);
					stack.push_back(e);
				} else { // e2 is in the opposite side
					// cerr << e->b << " " << e2->b << " right opp" << endl;
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
					// cerr << e->b << " " << e2->b << " left same" << endl;
					do {
						stack.pop_back();
						if(stack.empty()) break;
						Edge *e3 = stack.back();
						if(turnLeft(vertices, e->b, e2->b, e3->b)) {
							addEdge(e, e3);
							e2 = e3;
						} else break;
					} while(!stack.empty());
					stack.push_back(e->next);
					stack.push_back(e);
				} else { // e2 is in the opposite side
					// cerr << e->b << " " << e2->b << " left opp" << endl;
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
		// cerr << e->b << " " << " end" << endl;
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