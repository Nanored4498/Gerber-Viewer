#include "vec2.h"

Vec2 operator*(double scale, const Vec2 &v) { return Vec2(scale * v.x, scale * v.y); }


double dot(const Vec2 &a, const Vec2 &b) { return a.x*b.x + a.y*b.y; }