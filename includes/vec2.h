#pragma once

class Vec2 {
public:
	double x, y;

	Vec2(double p_x=0., double p_y=0.): x(p_x), y(p_y) {}

	inline Vec2 operator+(const Vec2 &other) const { return Vec2(x + other.x, y + other.y); }
	inline Vec2 operator-(const Vec2 &other) const { return Vec2(x - other.x, y - other.y); }
	inline Vec2& operator+=(const Vec2 &other) {
		x += other.x;
		y += other.y;
		return *this;
	}
	inline Vec2& operator-=(const Vec2 &other) {
		x -= other.x;
		y -= other.y;
		return *this;
	}

	inline Vec2 operator/(double scalar) const { return Vec2(x / scalar, y / scalar); }
	inline Vec2& operator/=(double scalar) {
		x /= scalar;
		y /= scalar;
		return *this;
	}

	inline bool operator!=(const Vec2 &other) { return x != other.x || y != other.y; }
};

Vec2 operator*(double scale, const Vec2 &v);