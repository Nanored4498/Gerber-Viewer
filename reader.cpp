#include "reader.h"

#include <iostream>
#include <vector>
#include <map>
#include <set>
#include <cmath>

#include "triangulate.h"

#include "glad.h"

using namespace std;

void skipLine(istream &in, char end='\n') {
	char c='0';
	while(c != end && c != EOF) in.get(c);
}

Object createObj(const vector<float> vertices, const vector<uint> indices) {
	uint VAO, VBO, EBO;
	glGenVertexArrays(1, &VAO);
	glBindVertexArray(VAO);

	glGenBuffers(1, &VBO);
	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float)*vertices.size(), vertices.data(), GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3*sizeof(float), 0);
	glEnableVertexAttribArray(0);

	glGenBuffers(1, &EBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(uint)*indices.size(), indices.data(), GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	float x0=1e9, x1=-1e9, y0=1e9, y1=-1e9;
	for(int i = 0; i < (int) vertices.size(); i += 3) {
		x0 = min(x0, vertices[i]);
		x1 = max(x1, vertices[i]);
		y0 = min(y0, vertices[i+1]);
		y1 = max(y1, vertices[i+1]);
	}

	return {VAO, VBO, EBO, indices.size(), x0, x1, y0, y1};
}

Object readXNC(istream &in) {
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
				uint ind = vertices.size() / 3;
				for(int i = 0; i < 24; ++i) {
					float a = 2*M_PI*i/24;
					vertices.push_back(x + r*cos(a));
					vertices.push_back(y + r*sin(a));
					vertices.push_back(0);
					indices.push_back(ind + i);
					indices.push_back(ind + ((i+1) % 24));
					indices.push_back(ind + 24);
				}
				vertices.push_back(x);
				vertices.push_back(y);
				vertices.push_back(0);
			}
		} else {
			cerr << "Unknown word: " << s << endl;
			skipLine(in);
		}
		in >> s;
	}

	return createObj(vertices, indices);
}

struct Aperture {
	string temp_name;
	vector<float> parameters;
};

Object readGerber(istream &in) {
	float SCALE = 1.0, DIV_X = 1.0, DIV_Y = 1.0;
	bool dark = true;
	map<int, Aperture> apertures;
	int ap_id = -1;
	char interpolation_mode = 1, quadrant = 0;
	bool in_region = false;
	float cX=0, cY=0;
	vector<float> vertices;
	vector<uint> indices;
	uint region_start = 0;

	string s;
	in >> s;
	while(s != "M02*") {
		if(s == "G04") skipLine(in, '*');
		else if(s.size() == 13 && s.substr(0, 6) == "%FSLAX" && s[8] == 'Y' && s[11] == '*' && s[12] == '%') {
			if(s[7] == '5') DIV_X = 1e-5;
			else if(s[7] == '6') DIV_X = 1e-6;
			else cerr << "X cordinates have to have 5 or 6 digits in the fractional part." << endl;
			if(s[10] == '5') DIV_Y = 1e-5;
			else if(s[10] == '6') DIV_Y = 1e-6;
			else cerr << "X cordinates have to have 5 or 6 digits in the fractional part." << endl;
		} else if(s.size() == 7 && s.substr(0, 3) == "%MO" && s[5] == '*' && s[6] == '%') {
			string unit = s.substr(3, 2);
			if(unit == "MM") SCALE = 1.0;
			else if(unit == "IN") SCALE = 25.4;
			else cerr << "Unknown unit: " << unit << endl;
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
					ap.parameters.push_back(stof(s.substr(start, param_ind-start)));
				}
				int np = ap.parameters.size();
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
			int id = atoi(s.substr(1, s.size()-2).c_str());
			if(apertures.count(id)) ap_id = id;
			else cerr << "The aperture number " << id << " is not defined" << endl;
		} else if(s == "G01*") interpolation_mode = 1;
		else if(s == "G02*") interpolation_mode = 2;
		else if(s == "G03*") interpolation_mode = 3;
		else if(s == "G36*") {
			if(in_region) cerr << "Can't open a region already opened !!" << endl;
			else {
				in_region = true;
				region_start = vertices.size() / 3;
			}
		} else if(s == "G37*") {
			if(!in_region) cerr << "Can't close a region without opening it !!" << endl;
			else {
				in_region = false;
				if(region_start+4 <= vertices.size()/3) {
					if(vertices[3*region_start] == vertices[vertices.size()-3] && vertices[3*region_start+1] == vertices[vertices.size()-2]) {
						vertices.pop_back();
						vertices.pop_back();
						vertices.pop_back();
						triangulate(vertices, indices, region_start);
					} else cerr << "The contour need to be closed when using command G37 !!";
				}
			}
		} else if(s.size() >= 4 && s[s.size()-4] == 'D' && s[s.size()-3] == '0' && s.back() == '*') {
			if(ap_id >= 0) {
				float x = cX, y = cY;
				if(s[0] == 'X') {
					size_t y_ind = s.find('Y');
					if(y_ind == string::npos) x = stof(s.substr(1, s.size()-5)) * SCALE * DIV_X;
					else {
						x = stof(s.substr(1, y_ind-1)) * SCALE * DIV_X;
						y = stof(s.substr(y_ind+1, s.size()-5-y_ind)) * SCALE * DIV_Y;
					}
				} else if(s[0] == 'Y') y = stof(s.substr(1, s.size()-5)) * SCALE * DIV_Y;
				char d = s[s.size()-2];
				if(d == '1') {
					if(in_region) {
						if(3*region_start == vertices.size()) cerr << "Need to define a fist point with D02 command before tracing a segment with D01 !!!" << endl;
						else {
							vertices.push_back(x);
							vertices.push_back(y);
							vertices.push_back(0);
						}
					} else {
						vector<float> &params = apertures[ap_id].parameters;
						if(apertures[ap_id].temp_name == "C") {
							if(params.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
							if(interpolation_mode == 1) {
								float r = .5*params[0];
								float a0 = atan2(y-cY, x-cX);
								uint ind;
								float xs[] = {cX, x}, ys[] = {cY, y};
								for(int j : {0, 1}) {
									ind = vertices.size() / 3;
									for(int i = 0; i <= 12; ++i) {
										float a = 2*M_PI*i/24 + M_PI_2 + a0 + j*M_PI;
										vertices.push_back(xs[j] + r*cos(a));
										vertices.push_back(ys[j] + r*sin(a));
										vertices.push_back(0);
										if(i < 12) {
											indices.push_back(ind + i);
											indices.push_back(ind + i+1);
											indices.push_back(ind + 13);
										}
									}
									vertices.push_back(xs[j]);
									vertices.push_back(ys[j]);
									vertices.push_back(0);
								}
								indices.push_back(ind-14);
								indices.push_back(ind-2);
								indices.push_back(ind);
								indices.push_back(ind-14);
								indices.push_back(ind);
								indices.push_back(ind+12);

							}
						}
					}
				} else if(d == '2') {
					if(in_region) {
						if(region_start+4 <= vertices.size()/3) {
							if(vertices[3*region_start] == vertices[vertices.size()-3] && vertices[3*region_start+1] == vertices[vertices.size()-2]) {
								vertices.pop_back();
								vertices.pop_back();
								vertices.pop_back();
								triangulate(vertices, indices, region_start);
							} else cerr << "The contour need to be closed when using command D02 !!";
						}
						region_start = vertices.size() / 3;
						vertices.push_back(x);
						vertices.push_back(y);
						vertices.push_back(0);
					}
				} else if(d == '3') {
					if(in_region) cerr << "Can't use operation D03 in a region statement: " << s << endl;
					else {
						vector<float> &params = apertures[ap_id].parameters;
						uint ind = vertices.size() / 3;
						if(apertures[ap_id].temp_name == "C") {
							float r = .5 * params[0];
							if(params.size() == 2) cerr << "Warning: holes are not implemented for circles" << endl;
							for(int i = 0; i < 24; ++i) {
								float a = 2*M_PI*i/24;
								vertices.push_back(x + r*cos(a));
								vertices.push_back(y + r*sin(a));
								vertices.push_back(0);
								indices.push_back(ind + i);
								indices.push_back(ind + ((i+1) % 24));
								indices.push_back(ind + 24);
							}
							vertices.push_back(x);
							vertices.push_back(y);
							vertices.push_back(0);
						} else if(apertures[ap_id].temp_name == "R") {
							float w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for rectangle" << endl;
							for(float dx : {-w, w}) for(float dy : {-h, h}) {
								vertices.push_back(x+dx);
								vertices.push_back(y+dy);
								vertices.push_back(0);
							}
							indices.push_back(ind);
							indices.push_back(ind+1);
							indices.push_back(ind+2);
							indices.push_back(ind+1);
							indices.push_back(ind+2);
							indices.push_back(ind+3);
						} else if(apertures[ap_id].temp_name == "O") {
							float w = .5*params[0], h = .5*params[1];
							if(params.size() == 3) cerr << "Warning: holes are not implemented for obrounds" << endl;
							float dhw = abs(h - w);
							bool vert = h > w;
							float r = vert ? w : h;
							float a0 = vert ? M_PI_2 : 0;
							for(int hc : {-1, 1}) {
								float xc = vert ? x : x + hc * dhw;
								float yc = vert ? y + hc * dhw : y;
								ind = vertices.size() / 3;
								for(int i = 0; i <= 12; ++i) {
									float a = 2*M_PI*i/24 - hc*M_PI_2 + a0;
									vertices.push_back(xc + r*cos(a));
									vertices.push_back(yc + r*sin(a));
									vertices.push_back(0);
									if(i < 12) {
										indices.push_back(ind + i);
										indices.push_back(ind + i+1);
										indices.push_back(ind + 13);
									}
								}
								vertices.push_back(xc);
								vertices.push_back(yc);
								vertices.push_back(0);
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
	
	return createObj(vertices, indices);
}