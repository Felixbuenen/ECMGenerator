#pragma once

#include "boost/polygon/voronoi.hpp"

#include "Configuration.h"

namespace ECM {

	struct Vec2 {
		Vec2() { }
		Vec2(float _x, float _y)
			: x(_x), y(_y) { }
		float x;
		float y;

		Vec2 operator+(const Vec2& a) const
		{
			return Vec2(a.x + x, a.y + y);
		}
		Vec2 operator-(const Vec2& a) const
		{
			return Vec2(x - a.x, y - a.y);
		}
		Vec2 operator *(const float& a) const {
			return Vec2(x * a, y * a);
		}
		Vec2 operator /(const float& a) const {
			return Vec2(x / a, y / a);
		}

		float Length() const
		{
			return sqrt(x * x + y * y);
		}

		void Normalize()
		{
			float l = Length();
			if (l == 0.0f) return;

			x /= l;
			y /= l;
		}
		
		Vec2 Normalized() const
		{
			float l = Length();
			if (l == 0.0f) return Vec2();

			return Vec2(x / l, y / l);
		}

		bool Approximate(const Vec2& a) const
		{
			return x < (a.x + Utility::EPSILON) && x >(a.x - Utility::EPSILON) && y < (a.y + Utility::EPSILON) && y >(a.y - Utility::EPSILON);
		}
	};

	struct Point {
		float x;
		float y;
		Point(float x, float y) : x(x), y(y) {}
		Point() { x = 0; y = 0; }
		Point(const Vec2& v) : x(v.x), y(v.y) {}

		bool Approximate(const Point& a) const
		{
			return x < (a.x + Utility::EPSILON) && x >(a.x - Utility::EPSILON) && y < (a.y + Utility::EPSILON) && y >(a.y - Utility::EPSILON);
		}

		operator Vec2() const { return Vec2(x, y); }

		Point operator =(const Point& a) {
			x = a.x;
			y = a.y;
			return *this;
		}
		Point operator -(const Point& a) const {
			return Point(x - a.x, y - a.y);
		}
		Point operator *(const float& a) const {
			return Point(x * a, y * a);
		}
		Point operator /(const float& a) const {
			return Point(x / a, y / a);
		}
		Point operator +(const Point& a) const {
			return Point(x + a.x, y + a.y);
		}
		Point operator +(const Vec2& a) const {
			return Point(x + a.x, y + a.y);
		}
		bool operator ==(const Point& a) const
		{
			using namespace Utility;
			return (x > a.x - EPSILON && x < a.x + EPSILON && y > a.y - EPSILON && y < a.y + EPSILON);
		}
	};

	struct Mat3x3
	{
		float row1[3];
		float row2[3];
		float row3[3];

		Mat3x3(float r1c1, float r1c2, float r1c3,
			float r2c1, float r2c2, float r2c3,
			float r3c1, float r3c2, float r3c3) :
			row1{ r1c1, r1c2, r1c3 }, 
			row2{r2c1, r2c2, r2c3}, 
			row3{r3c1, r3c2, r3c3} { }

		Vec2 operator -(const Vec2& a) const {
			float x = row1[0] * a.x + row1[1] * a.y + row1[2];
			float y = row2[0] * a.x + row2[1] * a.y + row2[2];
			float z = row3[0] * a.x + row3[1] * a.y + row3[2];

			x /= z;
			y /= z;

			return Vec2(x, y);
		}

		Mat3x3 operator *(const Mat3x3& a) const {
			return Mat3x3(row1[0] * a.row1[0] + row1[1] * a.row2[0] + row1[2] * a.row3[0],
				row1[0] * a.row1[1] + row1[1] * a.row2[1] + row1[2] * a.row3[1],
				row1[0] * a.row1[2] + row1[1] * a.row2[2] + row1[2] * a.row3[2],

				row2[0] * a.row1[0] + row2[1] * a.row2[0] + row2[2] * a.row3[0],
				row2[0] * a.row1[1] + row2[1] * a.row2[1] + row2[2] * a.row3[1],
				row2[0] * a.row1[2] + row2[1] * a.row2[2] + row2[2] * a.row3[2],

				row3[0] * a.row1[0] + row3[1] * a.row2[0] + row3[2] * a.row3[0],
				row3[0] * a.row1[1] + row3[1] * a.row2[1] + row3[2] * a.row3[1],
				row3[0] * a.row1[2] + row3[1] * a.row2[2] + row3[2] * a.row3[2]
			);
		}

		Mat3x3 Identity()
		{
			return Mat3x3(1, 0, 0, 0, 1, 0, 0, 0, 1);
		}
	};

	struct Segment {
		Point p0;
		Point p1;
		Segment() { }
		Segment(float x1, float y1, float x2, float y2) : p0(x1, y1), p1(x2, y2) {}
		Segment(Point point0, Point point1) : p0(point0), p1(point1) {}
	};



	struct BBOX {
		Point min;
		Point max;
		BBOX(Point min, Point max) : min(min), max(max) {}
		BBOX()
		{
			using Utility::MAX_FLOAT;
			using Utility::MIN_FLOAT;

			min.x = MAX_FLOAT;
			min.y = MAX_FLOAT;
			max.x = MIN_FLOAT;
			max.y = MIN_FLOAT;
		}
	};

	class MedialAxis
	{
	public:
		boost::polygon::voronoi_diagram<double> VD;

	};

}