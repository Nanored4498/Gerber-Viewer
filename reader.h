#pragma once

#include <istream>

#include "object.h"

void readXNC(std::istream &in, float color0[3], PCB &pcb);
void readGerber(std::istream &in, float color0[3], PCB &pcb);