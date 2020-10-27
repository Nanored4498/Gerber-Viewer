#pragma once

#include <istream>

#include "object.h"

void readXNC(std::istream &in, float color0[3], std::vector<Object> &objs);
void readGerber(std::istream &in, float color0[3], std::vector<Object> &objs, std::vector<Path> &paths);