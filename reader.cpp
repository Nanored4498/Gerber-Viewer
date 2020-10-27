#include "reader.h"

#include <iostream>
#include <map>

#include "triangulate.h"

#define JC_VORONOI_IMPLEMENTATION
#include "jc_voronoi.h"

#include "glad.h"

using namespace std;

void skipLine(istream &in, char end='\n') {
	char c='0';
	while(c != end && c != EOF) in.get(c);
}

void readXNC(std::istream &in, float color0[3], vector<Object> &objs) {
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
				objs.emplace_back(vertices, indices, color0);
			}
		} else {
			cerr << "Unknown word: " << s << endl;
			skipLine(in);
		}
		in >> s;
	}
}

struct PrePath {
	float x0, y0;
	vector<float> inter;
	Aperture ap;
	int interpolation_mode;
	PrePath(float p_x0, float p_y0, const Aperture &p_ap, int mode):
		x0(p_x0), y0(p_y0), ap(p_ap), interpolation_mode(mode) {}
};

void readGerber(std::istream &in, float color0[3], vector<Object> &objs, vector<Path> &paths) {
	double DIV_X = 1.0, DIV_Y = 1.0;
	bool dark = true;
	map<int, Aperture> apertures;
	int ap_id = -1;
	char interpolation_mode = 1, quadrant = 0;
	bool in_region = false;
	long long cX=0, cY=0;
	vector<long long> coords;
	vector<float> vertices;
	vector<PrePath> pre_paths;
	vector<uint> indices;
	bool in_path = false;

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
				objs.emplace_back(vertices, indices, color0);
			} else cerr << "The contour need to be closed when using command G37 !!";
		}
		coords.clear();
	};

	uint obj_start = objs.size();

	string s;
	in >> s;
	while(s != "M02*" && s != "M00*") {
		bool pred_path = in_path;
		in_path = false;
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
							if(apertures[ap_id].temp_name != "C")
								cerr << "The aperture " << apertures[ap_id].temp_name << " is not implemented for interpolations" << endl;
							else {
								if(apertures[ap_id].parameters.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
								in_path = true;
								if(!pred_path) pre_paths.emplace_back(cX/DIV_X, cY/DIV_Y, apertures[ap_id], interpolation_mode);
								pre_paths.back().inter.push_back(xx);
								pre_paths.back().inter.push_back(yy);
							}
						} else cerr << "The interpolation mode " << interpolation_mode << " is not supported yet !" << endl;
					}
				} else if(d == '2') {
					if(in_region) {
						close_region();
						coords.push_back(x);
						coords.push_back(y);
					}
					// if(pred_path && cX == x && cY == y) in_path = true;
				} else if(d == '3') {
					if(in_region) cerr << "Can't use operation D03 in a region statement: " << s << endl;
					else {
						vertices.clear();
						indices.clear();
						vector<double> &params = apertures[ap_id].parameters;
						if(apertures[ap_id].temp_name == "C") {
							double r = .5 * params[0];
							if(params.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
							for(uint i = 0; i < 24; ++i) {
								double a = (2*M_PI*i)/24;
								vertices.push_back(xx + r*cos(a));
								vertices.push_back(yy + r*sin(a));
								indices.push_back(i);
								indices.push_back(((i+1) % 24));
								indices.push_back(24);
							}
							vertices.push_back(xx);
							vertices.push_back(yy);
						} else if(apertures[ap_id].temp_name == "R") {
							double w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for rectangle" << endl;
							for(double dx : {-w, w}) for(double dy : {-h, h}) {
								vertices.push_back(xx + dx);
								vertices.push_back(yy + dy);
							}
							indices.push_back(0);
							indices.push_back(1);
							indices.push_back(2);
							indices.push_back(1);
							indices.push_back(2);
							indices.push_back(3);
						} else if(apertures[ap_id].temp_name == "O") {
							double w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for obrounds" << endl;
							double dhw = abs(h - w);
							bool vert = h > w;
							double r = vert ? w : h;
							double a0 = vert ? M_PI_2 : 0;
							for(int hc : {-1, 1}) {
								double xc = vert ? xx : xx + hc * dhw;
								double yc = vert ? yy + hc * dhw : yy;
								uint ind = vertices.size()/2;
								for(int i = 0; i <= 12; ++i) {
									double a = (2*M_PI*i)/24 - hc*M_PI_2 + a0;
									vertices.push_back(xc + r*cos(a));
									vertices.push_back(yc + r*sin(a));
									if(i < 12) {
										indices.push_back(ind + i);
										indices.push_back(ind + i+1);
										indices.push_back(ind + 13);
									}
								}
								vertices.push_back(xc);
								vertices.push_back(yc);
							}
							indices.push_back(0);
							indices.push_back(12);
							indices.push_back(14);
							indices.push_back(0);
							indices.push_back(14);
							indices.push_back(26);
						}
						if(!indices.empty()) objs.emplace_back(vertices, indices, color0);
					}
				} else cerr << "Unknown operation: " << d << endl;
				cX = x;
				cY = y;
			} else cerr << "No tool selected while executing the command: " << s << endl;
		} else cerr << "Unknown word: " << s << endl;
		in >> s;
	}

	// Paths
	const auto find_nearest = [&](float x, float y)->uint {
		double bd = 1e9;
		uint bi = objs.size();
		for(uint i = obj_start; i < objs.size(); ++i) {
			double dx = x - objs[i].center[0];
			double dy = y - objs[i].center[1];
			double d = sqrt(dx*dx + dy*dy);
			if(d < bd) {
				bi = i;
				bd = d;
			}
		}
		if(bi == objs.size()) return objs.size();
		float x0=objs[bi].center[0], x1=objs[bi].center[0], y0=objs[bi].center[1], y1=objs[bi].center[1];
		objs[bi].update_bounding_box(x0, x1, y0, y1);
		if(x < x0 || x > x1 || y < y0 || y > y1) {
			// cerr << "not in the box " << x0 << " " << x1 << " " << y0 << " " << y1 << " " << x << " " << y << endl;
			return objs.size();
		}
		return bi;
	};
	for(PrePath &pp : pre_paths) {
		uint ai = find_nearest(pp.x0, pp.y0);
		if(ai == objs.size()) continue;
		float y = pp.inter.back();
		pp.inter.pop_back();
		float x = pp.inter.back();
		pp.inter.pop_back();
		uint bi = find_nearest(x, y);
		if(bi == objs.size() || bi == ai) continue;
		paths.emplace_back(objs[ai], objs[bi], pp.inter, pp.ap, pp.interpolation_mode);
	}
}