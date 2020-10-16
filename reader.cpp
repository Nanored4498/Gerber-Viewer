#include "reader.h"

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <cmath>
#include <random>

#include "triangulate.h"

#define JC_VORONOI_IMPLEMENTATION
#include "jc_voronoi.h"

#include "glad.h"

using namespace std;

uniform_real_distribution<float> unif(0, 1);
default_random_engine re;

void skipLine(istream &in, char end='\n') {
	char c='0';
	while(c != end && c != EOF) in.get(c);
}

Object createObj(const vector<float> vertices, const vector<uint> indices, const vector<float> colors) {
	uint VAO, VBO, VBOc, EBO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2*sizeof(float), 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &VBOc);
	glBindBuffer(GL_ARRAY_BUFFER, VBOc);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*colors.size(), colors.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), 0);
	glEnableVertexAttribArray(1);

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*indices.size(), indices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	float x0=1e9, x1=-1e9, y0=1e9, y1=-1e9;
	for(int i = 0; i < (int) vertices.size(); i += 2) {
		x0 = min(x0, vertices[i]);
		x1 = max(x1, vertices[i]);
		y0 = min(y0, vertices[i+1]);
		y1 = max(y1, vertices[i+1]);
	}

	return {VAO, VBO, VBOc, EBO, indices.size(), x0, x1, y0, y1};
}

Object readXNC(istream &in, float *color0) {
	float SCALE = 1.0;
	float tools[100];
	for(int t = 0; t < 100; ++t) tools[t] = -1;
	bool incremental = false;
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
		if(s == "G90") incremental = false;
		else if(s == "G91") incremental = true;
		else if(s == "G00") drill = false;
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
				float x = stof(s.substr(1, i-1)) * SCALE;
				float y = stof(s.substr(i+1)) * SCALE;
				float r = .5 * tools[T];
				uint ind = vertices.size() / 2;
				for(int i = 0; i < 24; ++i) {
					float a = 2*M_PI*i/24;
					vertices.push_back(x + r*cos(a));
					vertices.push_back(y + r*sin(a));
					indices.push_back(ind + i);
					indices.push_back(ind + ((i+1) % 24));
					indices.push_back(ind + 24);
				}
				vertices.push_back(x);
				vertices.push_back(y);
			}
		} else {
			cerr << "Unknown word: " << s << endl;
			skipLine(in);
		}
		in >> s;
	}

	vector<float> colors(3*vertices.size()/2);
	for(int i = 0; i < colors.size(); ++i) colors[i] = color0[i%3];

	return createObj(vertices, indices, colors);
}

struct Aperture {
	string temp_name;
	vector<double> parameters;
};

pair<Object, Object> readGerber(istream &in, float *color0) {
	double DIV_X = 1.0, DIV_Y = 1.0;
	bool dark = true;
	map<int, Aperture> apertures;
	int ap_id = -1;
	char interpolation_mode = 1, quadrant = 0;
	bool in_region = false;
	long long cX=0, cY=0;
	vector<long long> coords;
	vector<int> group;
	vector<uint> indices;
	uint region_start = 0;

	const auto add_point = [&](long long x, long long y, int g) {
		coords.push_back(x);
		coords.push_back(y);
		group.push_back(g);
	};

	string s;
	in >> s;
	while(s != "M02*" && s != "M00*") {
		int g = group.empty() ? 0 : group.back()+1;
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
			if(polarity == 'D') dark = true;
			else if(polarity == 'C') {
				dark = false;
				cerr << "Warning: Clear polarity mode is not implemented !!" << endl;
			} else cerr << "Unknown polarity: " << polarity << endl;
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
				if(good) apertures[aper_id] = ap;
			}
		} else if(s[0] == 'D' && s.back() == '*') {
			int id = stoi(s.substr(1, s.size()-2));
			if(apertures.count(id)) ap_id = id;
			else cerr << "The aperture number " << id << " is not defined" << endl;
		} else if(s.substr(0, 4) == "G54D" && s.back() == '*') {
			int id = stoi(s.substr(4, s.size()-5));
			if(apertures.count(id)) ap_id = id;
			else cerr << "The aperture number " << id << " is not defined" << endl;
		} else if(s == "G01*") interpolation_mode = 1;
		else if(s == "G02*") interpolation_mode = 2;
		else if(s == "G03*") interpolation_mode = 3;
		else if(s == "G36*") {
			if(in_region) cerr << "Can't open a region already opened !!" << endl;
			else {
				in_region = true;
				region_start = coords.size() / 2;
			}
		} else if(s == "G37*") {
			if(!in_region) cerr << "Can't close a region without opening it !!" << endl;
			else {
				in_region = false;
				if(region_start+4 <= coords.size()/2) {
					if(coords[2*region_start] == coords[coords.size()-2] && coords[2*region_start+1] == coords.back()) {
						coords.pop_back();
						coords.pop_back();
						group.pop_back();
						triangulate(coords, indices, region_start);
					} else cerr << "The contour need to be closed when using command G37 !!";
				} else while(coords.size() > region_start*2) {
					coords.pop_back();
					coords.pop_back();
					group.pop_back();
				}
			}
		} else if(s == "G90*") {
			// By default
		} else if(s == "G91*") {
			cerr << "Incremental notations is not supported !!" << endl;
		} else if(s.size() >= 4 && s[s.size()-4] == 'D' && s[s.size()-3] == '0' && s.back() == '*') {
			if(ap_id >= 0 || in_region) {
				long long x = cX, y = cY;
				for(auto p : {make_pair('X', &x), make_pair('Y', &y)}) {
					size_t ind = s.find(p.first);
					if(ind != string::npos) {
						size_t i = ++ ind;
						if(s[i] == '-') ++ i;
						while(s[i] >= '0' && s[i] <= '9') ++i;
						*p.second = stoll(s.substr(ind, i-ind));
					}
				}
				char d = s[s.size()-2];
				if(d == '1') {
					if(in_region) {
						if(2*region_start == coords.size()) cerr << "Need to define a fist point with D02 command before tracing a segment with D01 !!!" << endl;
						else add_point(x, y, group.back());
					} else {
						vector<double> &params = apertures[ap_id].parameters;
						if(apertures[ap_id].temp_name == "C") {
							// if(params.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
							// if(interpolation_mode == 1) {
							// 	double r = .5*params[0];
							// 	double a0 = atan2(y-cY, x-cX);
							// 	uint ind;
							// 	long long xs[] = {cX, x}, ys[] = {cY, y};
							// 	for(int j : {0, 1}) {
							// 		ind = coords.size() / 2;
							// 		for(int i = 0; i <= 12; ++i) {
							// 			double a = 2*M_PI*i/24 + M_PI_2 + a0 + j*M_PI;
							// 			add_point(xs[j] + r*cos(a)*DIV_X, ys[j] + r*sin(a)*DIV_Y, g);
							// 			if(i < 12) {
							// 				indices.push_back(ind + i);
							// 				indices.push_back(ind + i+1);
							// 				indices.push_back(ind + 13);
							// 			}
							// 		}
							// 		add_point(xs[j], ys[j], g);
							// 	}
							// 	indices.push_back(ind-14);
							// 	indices.push_back(ind-2);
							// 	indices.push_back(ind);
							// 	indices.push_back(ind-14);
							// 	indices.push_back(ind);
							// 	indices.push_back(ind+12);
							// }
						} else cerr << "The aperture " << apertures[ap_id].temp_name << " is not implemented for interpolations" << endl;
					}
				} else if(d == '2') {
					if(in_region) {
						if(region_start+4 <= coords.size()/2) {
							if(coords[2*region_start] == coords[coords.size()-2] && coords[2*region_start+1] == coords.back()) {
								coords.pop_back();
								coords.pop_back();
								group.pop_back();
								triangulate(coords, indices, region_start);
							} else cerr << "The contour need to be closed when using command D02 !!";
						} else while(coords.size() > region_start*2) {
							coords.pop_back();
							coords.pop_back();
							group.pop_back();
						}
						region_start = coords.size() / 2;
						add_point(x, y, g);
					}
				} else if(d == '3') {
					if(in_region) cerr << "Can't use operation D03 in a region statement: " << s << endl;
					else {
						vector<double> &params = apertures[ap_id].parameters;
						uint ind = coords.size() / 2;
						if(apertures[ap_id].temp_name == "C") {
							double r = .5 * params[0];
							if(params.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
							for(int i = 0; i < 24; ++i) {
								double a = 2*M_PI*i/24;
								add_point(x + r*cos(a)*DIV_X, y + r*sin(a)*DIV_Y, g);
								indices.push_back(ind + i);
								indices.push_back(ind + ((i+1) % 24));
								indices.push_back(ind + 24);
							}
							add_point(x, y, g);
						} else if(apertures[ap_id].temp_name == "R") {
							long long w = .5*params[0]*DIV_X, h = .5*params[1]*DIV_Y;
							if(params.size() == 3) cerr << "Warning: holes are not implemented for rectangle" << endl;
							for(long long dx : {-w, w}) for(double dy : {-h, h}) add_point(x + dx, y + dy, g);
							indices.push_back(ind);
							indices.push_back(ind+1);
							indices.push_back(ind+2);
							indices.push_back(ind+1);
							indices.push_back(ind+2);
							indices.push_back(ind+3);
						} else if(apertures[ap_id].temp_name == "O") {
							double w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for obrounds" << endl;
							double dhw = abs(h - w);
							bool vert = h > w;
							double r = vert ? w : h;
							double a0 = vert ? M_PI_2 : 0;
							for(int hc : {-1, 1}) {
								double xc = vert ? x : x + hc * dhw * DIV_X;
								double yc = vert ? y + hc * dhw * DIV_Y : y;
								ind = coords.size() / 2;
								for(int i = 0; i <= 12; ++i) {
									double a = 2*M_PI*i/24 - hc*M_PI_2 + a0;
									add_point(xc + r*cos(a)*DIV_X, yc + r*sin(a)*DIV_Y, g);
									if(i < 12) {
										indices.push_back(ind + i);
										indices.push_back(ind + i+1);
										indices.push_back(ind + 13);
									}
								}
								add_point(xc, yc, g);
							}
							indices.push_back(ind-14);
							indices.push_back(ind-2);
							indices.push_back(ind);
							indices.push_back(ind-14);
							indices.push_back(ind);
							indices.push_back(ind+12);
						}
					}
				} else cerr << "Unknown operation: " << d << endl;
				cX = x;
				cY = y;
			} else cerr << "No tool selected while executing the command: " << s << endl;
		} else cerr << "Unknown word: " << s << endl;
		in >> s;
	}
	
	vector<float> cg(3*(group.back()+1));
	for(float &c : cg) c = unif(re);
	uint N = coords.size()/2;
	vector<float> vertices(2*N, 0.);
	for(uint i = 0; i < N; ++i) {
		vertices[2*i] = coords[2*i] / DIV_X;
		vertices[2*i+1] = coords[2*i+1] / DIV_Y;
	}
	vector<float> colors(3*N);
	for(int i = 0; i < N; ++i) for(int c = 0; c < 3; ++c) colors[3*i+c] = .6*cg[3*group[i]+c];
	Object ans = createObj(vertices, indices, colors);

	vertices.clear();
	indices.clear();
	colors.clear();
	jcv_point points[N];
	for(uint i = 0; i < N; ++i) points[i] = {vertices[2*i], vertices[2*i+1]};
	jcv_rect rect = {{ans.x0 - .1*(ans.x1-ans.x0), ans.y0 - .1*(ans.y1-ans.y0)},
					{ans.x1 + .1*(ans.x1-ans.x0), ans.y1 + .1*(ans.y1-ans.y0)}};
	jcv_diagram diagram;
	memset(&diagram, 0, sizeof(jcv_diagram));
	jcv_diagram_generate(N, points, &rect, nullptr, &diagram);
	const jcv_site* sites = jcv_diagram_get_sites(&diagram);
	for(int i = 0; i < diagram.numsites; ++i) {
		const jcv_site* site = &sites[i];
		const jcv_graphedge* e = site->edges;
		int g = group[site->index];
		uint si = vertices.size()/2;
		vertices.push_back(site->p.x);
		vertices.push_back(site->p.y);
		for(int c = 0; c < 3; ++c) colors.push_back(cg[3*g+c]);
		while(e) {
			uint j = vertices.size()/2;
			vertices.push_back(e->pos[0].x);
			vertices.push_back(e->pos[0].y);
			for(int c = 0; c < 3; ++c) colors.push_back(cg[3*g+c]);
			vertices.push_back(e->pos[1].x);
			vertices.push_back(e->pos[1].y);
			for(int c = 0; c < 3; ++c) colors.push_back(cg[3*g+c]);
			indices.push_back(si);
			indices.push_back(j);
			indices.push_back(j+1);
			e = e->next;
		}
	}
	jcv_diagram_free(&diagram);

	return {createObj(vertices, indices, colors), ans};
}