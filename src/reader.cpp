#include "reader.h"

#include <iostream>
#include <map>
#include <algorithm>
#include <cmath>

#include "triangulate.h"

using namespace std;
typedef long long LL;

void skipLine(istream &in, char end='\n') {
	char c='0';
	while(c != end && c != EOF) in.get(c);
}

void readXNC(std::istream &in, float color0[3], PCB &pcb) {
	float SCALE = 1.0;
	float tools[100];
	for(int t = 0; t < 100; ++t) tools[t] = -1;
	bool drill = true;
	vector<float> vertices;
	vector<uint> indices;

	// header
	string s;
	in >> s;
	if(s == "M48") {
		while(true) {
			in >> s;
			if(s == "%") break;
			else if(s == ";") skipLine(in);
			else if(s == "METRIC") SCALE = 1.0;
			else if(s == "INCH") SCALE = 25.4;
			else if(s[0] == 'T') {
				size_t i = s.find('C');
				if(i == string::npos) {
					cerr << "Unknown word: " << s << endl;
					skipLine(in);
				} else {
					int t = stoi(s.substr(1, i-1));
					if(t < 0 || t >= 100) cerr << "Tool index has to be between 0 and 99" << endl;
					else tools[t] = stof(s.substr(i+1)) * SCALE;
				}
			} else {
				cerr << "Unknown word: " << s << endl;
				skipLine(in);
			} 
		}
		in >> s;
	}

	// body
	int T = -1;
	while(s != "M30") {
		if(s == "G00") drill = false;
		else if(s == "G05") drill = true;
		else if(s[0] == 'T') {
			int t = atoi(s.c_str()+1);
			if(t < 0 || t >= 100) cerr << "Tool index has to be between 0 and 99 (you gave: " << t << ")" << endl;
			else if(tools[t] < 0) cerr << "The tool " << t << " is not defined" << endl;
			else T = t;
		} else if(s[0] == 'X') {
			size_t i = s.find('Y');
			if(i == string::npos) {
				cerr << "Unknown word: " << s << endl;
				skipLine(in);
			} else if(!drill) cerr << "Drill mode required for the command: " << s << endl;
			else if(T < 0) cerr << "No tool selected while executing the command: " << s << endl;
			else {
				vertices.clear();
				indices.clear();
				float x = stof(s.substr(1, i-1)) * SCALE;
				float y = stof(s.substr(i+1)) * SCALE;
				float r = .5 * tools[T];
				for(uint i = 0; i < 24; ++i) {
					float a = (2*M_PI*i)/24;
					vertices.push_back(x + r*cos(a));
					vertices.push_back(y + r*sin(a));
					indices.push_back(i);
					indices.push_back(((i+1) % 24));
					indices.push_back(24);
				}
				vertices.push_back(x);
				vertices.push_back(y);
				pcb.objs.emplace_back(vertices, indices, color0);
			}
		} else {
			cerr << "Unknown word: " << s << endl;
			skipLine(in);
		}
		in >> s;
	}
}

struct Edge {
	pair<LL, LL> from, to;
	int ap;
	int interpolation_mode;
	Edge(const pair<LL, LL> &p_from, const pair<LL, LL> &p_to, int p_ap, int p_mode):
		from(p_from), to(p_to), ap(p_ap), interpolation_mode(p_mode) {}
};

void readGerber(std::istream &in, float color0[3], PCB &pcb) {
	double DIV_X = 1.0, DIV_Y = 1.0;
	int ap_id = -1;
	char interpolation_mode = 1;
	bool in_region = false;
	LL cX=0, cY=0;
	map<int, uint> map_ap;
	vector<LL> coords;
	vector<float> vertices;
	vector<uint> indices;
	map<pair<LL, LL>, int> convert;
	vector<Edge> edges;

	uint obj_start = pcb.objs.size();

	const auto close_region = [&]() {
		if(coords.size() >= 8) {
			if(coords[0] == coords[coords.size()-2] && coords[1] == coords.back()) {
				coords.pop_back();
				coords.pop_back();
				triangulate(coords, indices);
				uint N = coords.size();
				vertices.resize(N);
				for(uint i = 0; i < N; i+=2) {
					vertices[i] = coords[i] / DIV_X;
					vertices[i+1] = coords[i+1] / DIV_Y;
				}
				pcb.objs.emplace_back(vertices, indices, color0);
			} else cerr << "The contour need to be closed when using command G37 !!";
		}
		coords.clear();
	};

	string s;
	in >> s;
	while(s != "M02*" && s != "M00*") {
		if(s == "G04") skipLine(in, '*');
		else if(s.size() == 13 && s.substr(0, 6) == "%FSLAX" && s[8] == 'Y' && s[11] == '*' && s[12] == '%') {
			if(s[7] == '4') DIV_X = 1e4;
			else if(s[7] == '5') DIV_X = 1e5;
			else if(s[7] == '6') DIV_X = 1e6;
			else cerr << "X cordinates have to have 4, 5 or 6 digits in the fractional part." << endl;
			if(s[10] == '4') DIV_Y = 1e4;
			else if(s[10] == '5') DIV_Y = 1e5;
			else if(s[10] == '6') DIV_Y = 1e6;
			else cerr << "X cordinates have to have 4, 5 or 6 digits in the fractional part." << endl;
		} else if(s.size() == 7 && s.substr(0, 3) == "%MO" && s[5] == '*' && s[6] == '%') {
			string unit = s.substr(3, 2);
			if(unit != "MM" && unit != "IN") cerr << "Unknown unit: " << unit << endl;
			// We don't care about the unit
		} else if(s.size() == 6 && s.substr(0, 3) == "%LP" && s[4] == '*' && s[5] == '%') {
			char polarity = s[3];
			if(polarity == 'C') {
				cerr << "Warning: Clear polarity mode is not implemented !!" << endl;
			} else if(polarity != 'D') cerr << "Unknown polarity: " << polarity << endl;
		} else if(s.substr(0, 4) == "%ADD" && s[s.size()-2] == '*' && s.back() == '%') {
			size_t temp_ind = 4;
			while(s[temp_ind] >= '0' && s[temp_ind] <= '9') ++ temp_ind;
			int aper_id = stoi(s.substr(4, temp_ind-4));
			if(aper_id < 10) cerr << "The aperture number " << aper_id << " is reserved and cannot be used" << endl;
			else {
				size_t param_ind = temp_ind;
				while(s[param_ind] != ',' && s[param_ind] != '*') ++ param_ind;
				Aperture ap = {s.substr(temp_ind, param_ind-temp_ind), {}};
				while(s[param_ind] != '*') {
					size_t start = ++ param_ind;
					while(s[param_ind] != 'X' && s[param_ind] != '*') ++ param_ind;
					ap.parameters.push_back(stod(s.substr(start, param_ind-start)));
				}
				uint np = ap.parameters.size();
				bool good = true;
				if(ap.temp_name == "C") {
					if(np < 1 || np > 2) {
						good = false;
						cerr << "The aperture " << ap_id << " is a circle and need to have 1 or 2 parameters. Not " << np << endl;
					}
				} else if(ap.temp_name == "R") {
					if(np < 2 || np > 3) {
						good = false;
						cerr << "The aperture " << ap_id << " is a rectangle and need to have 2 or 3 parameters. Not " << np << endl;
					}
				} else if(ap.temp_name == "O") {
					if(np < 2 || np > 3) {
						good = false;
						cerr << "The aperture " << ap_id << " is an obround and need to have 2 or 3 parameters. Not " << np << endl;
					}
				} else if(ap.temp_name == "P")  {
					if(np < 2 || np > 4) {
						good = false;
						cerr << "The aperture " << ap_id << " is an obround and need to have 2, 3 or 4 parameters. Not " << np << endl;
					}
				}
				if(good) {
					if(ap.temp_name == "O" && ap.parameters[0] == ap.parameters[1]) {
						ap.temp_name = "C";
						if(np == 3) ap.parameters[1] = ap.parameters[2];
						ap.parameters.pop_back();
					}
					map_ap[aper_id] = pcb.apertures.size();
					pcb.apertures.push_back(ap);
				}
			}
		} else if((s[0] == 'D' && s.back() == '*') || (s.substr(0, 4) == "G54D" && s.back() == '*')) {
			int id = stoi(s.substr(1, s.size()-2));
			if(map_ap.count(id)) ap_id = map_ap[id];
			else cerr << "The aperture number " << id << " is not defined" << endl;
		} else if(s == "G01*") interpolation_mode = 1;
		else if(s == "G02*") interpolation_mode = 2;
		else if(s == "G03*") interpolation_mode = 3;
		else if(s == "G36*") {
			if(in_region) cerr << "Can't open a region already opened !!" << endl;
			else in_region = true;
		} else if(s == "G37*") {
			if(!in_region) cerr << "Can't close a region without opening it !!" << endl;
			else {
				in_region = false;
				close_region();
			}
		} else if(s == "G90*") {
			// By default
		} else if(s == "G91*") {
			cerr << "Incremental notations is not supported !!" << endl;
		} else if(s.size() >= 4 && s[s.size()-4] == 'D' && s[s.size()-3] == '0' && s.back() == '*') {
			if(ap_id >= 0 || in_region) {
				LL x = cX, y = cY;
				for(auto p : {make_pair('X', &x), make_pair('Y', &y)}) {
					size_t ind = s.find(p.first);
					if(ind != string::npos) {
						size_t i = ++ ind;
						if(s[i] == '-') ++ i;
						while(s[i] >= '0' && s[i] <= '9') ++i;
						*p.second = stoll(s.substr(ind, i-ind));
					}
				}
				double xx = x/DIV_X, yy = y/DIV_Y;
				char d = s[s.size()-2];
				if(d == '1') {
					if(in_region) {
						if(coords.empty()) cerr << "Need to define a fist point with D02 command before tracing a segment with D01 !!!" << endl;
						else {
							coords.push_back(x);
							coords.push_back(y);
						}
					} else { // outside region !!!!!!
						vertices.clear();
						indices.clear();
						if(interpolation_mode == 1) {
							if(pcb.apertures[ap_id].temp_name != "C")
								cerr << "The aperture " << pcb.apertures[ap_id].temp_name << " is not implemented for interpolations" << endl;
							else {
								if(pcb.apertures[ap_id].parameters.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
								pair<LL, LL> from = {cX, cY}, to = {x, y};
								convert[from] = convert[to] = 0;
								edges.emplace_back(from, to, ap_id, interpolation_mode);
							}
						} else cerr << "The interpolation mode " << interpolation_mode << " is not supported yet !" << endl;
					}
				} else if(d == '2') {
					if(in_region) {
						close_region();
						coords.push_back(x);
						coords.push_back(y);
					}
				} else if(d == '3') {
					if(in_region) cerr << "Can't use operation D03 in a region statement: " << s << endl;
					else {
						vertices.clear();
						indices.clear();
						vector<double> &params = pcb.apertures[ap_id].parameters;
						if(pcb.apertures[ap_id].temp_name == "C") {
							double r = .5 * params[0];
							if(params.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
							for(uint i = 0; i < 24; ++i) {
								double a = (2*M_PI*i)/24;
								vertices.push_back(xx + r*cos(a));
								vertices.push_back(yy + r*sin(a));
								if(i > 1) {
									indices.push_back(0);
									indices.push_back(i-1);
									indices.push_back(i);
								}
							}
						} else if(pcb.apertures[ap_id].temp_name == "R") {
							double w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for rectangle" << endl;
							vertices.push_back(xx - w); vertices.push_back(yy - h);
							vertices.push_back(xx + w); vertices.push_back(yy - h);
							vertices.push_back(xx + w); vertices.push_back(yy + h);
							vertices.push_back(xx - w); vertices.push_back(yy + h);
							indices.push_back(0); indices.push_back(1); indices.push_back(2);
							indices.push_back(0); indices.push_back(2); indices.push_back(3);
						} else if(pcb.apertures[ap_id].temp_name == "O") {
							double w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for obrounds" << endl;
							double dhw = abs(h - w);
							bool vert = h > w;
							double r = vert ? w : h;
							double a0 = vert ? M_PI_2 : 0;
							for(int hc : {-1, 1}) {
								double xc = vert ? xx : xx + hc * dhw;
								double yc = vert ? yy + hc * dhw : yy;
								for(int i = 0; i <= 12; ++i) {
									double a = (2*M_PI*i)/24 - hc*M_PI_2 + a0;
									vertices.push_back(xc + r*cos(a));
									vertices.push_back(yc + r*sin(a));
								}
							}
							for(uint i = 2; i < vertices.size()/2; ++i) {
								indices.push_back(0);
								indices.push_back(i-1);
								indices.push_back(i);
							}
						}
						if(!indices.empty()) pcb.objs.emplace_back(vertices, indices, color0);
					}
				} else cerr << "Unknown operation: " << d << endl;
				cX = x;
				cY = y;
			} else cerr << "No tool selected while executing the command: " << s << endl;
		} else cerr << "Unknown word: " << s << endl;
		in >> s;
	}

	// Paths
	for(pair<const pair<LL, LL>, int> &p : convert) {
		float x = p.first.first/DIV_X, y = p.first.second/DIV_Y;
		double bd = 1e9;
		p.second = -1;
		for(int i = obj_start; i < (int) pcb.objs.size(); ++i) {
			double dx = x - pcb.objs[i].center[0];
			double dy = y - pcb.objs[i].center[1];
			double d = sqrt(dx*dx + dy*dy);
			if(d < bd) {
				p.second = i;
				bd = d;
			}
		}
		if(p.second >= 0) {
			float x0 = pcb.objs[p.second].center[0], x1 = pcb.objs[p.second].center[0];
			float y0 = pcb.objs[p.second].center[1], y1 = pcb.objs[p.second].center[1];
			pcb.objs[p.second].update_bounding_box(x0, x1, y0, y1);
			if(x < x0 || x > x1 || y < y0 || y > y1) p.second = -1;
		}
		if(p.second == -1) {
			p.second = -pcb.junctions.size()-1;
			pcb.junctions.push_back(x);
			pcb.junctions.push_back(y);
		}
	}
	uint i = 0;
	while(i < edges.size()) {
		if(convert[edges[i].from] == convert[edges[i].to]) {
			swap(edges[i], edges.back());
			edges.pop_back();
		} else {
			if(convert[edges[i].from] > convert[edges[i].to]) swap(edges[i].from, edges[i].to);
			++ i;
		}
	}
	sort(edges.begin(), edges.end(), [&](const Edge &a, const Edge &b) {
		int ia = convert[a.from], ib = convert[b.from];
		if(ia == ib) {
			ia = convert[a.to], ib = convert[b.to];
			if(ia == ib) {
				return a.ap < b.ap || (a.ap == b.ap && a.interpolation_mode < b.interpolation_mode);
			} else return ia < ib;
		} else return ia < ib;
	});
	i = 1;
	while(i < edges.size()) {
		if(convert[edges[i].from] == convert[edges[i-1].from]
				&& convert[edges[i].to] == convert[edges[i-1].to]
				&& edges[i].ap == edges[i-1].ap
				&& edges[i].interpolation_mode == edges[i-1].interpolation_mode) {
			swap(edges[i], edges.back());
			edges.pop_back();
		} else ++ i;
	}
	for(const Edge &e : edges)
		pcb.edges.emplace_back(convert[e.from], convert[e.to], e.ap, e.interpolation_mode);
}