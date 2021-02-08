#include "vec2.h"

Vec2 operator*(double scale, const Vec2 &v) { return Vec2(scale * v.x, scale * v.y); }