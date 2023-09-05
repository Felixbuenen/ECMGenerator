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
		Vec2 operator *(const float& a) const {
			return Vec2(x * a, y * a);
		}

		float Length()
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
	};

	struct Point {
		float x;
		float y;
		Point(float x, float y) : x(x), y(y) {}
		Point() { x = 0; y = 0; }

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

	struct Segment {
		Point p0;
		Point p1;
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
		// TODO: we want to avoid using Boost code all over the place. This medial axis class is a data container for the 
		// medial axis and provides functionality to use the medial axis (eg iterators, updating).
		// Probably do the same thing with ECMVertex and ECMSegment (this could be structs, containing e.g. coordinates and closest obstacle information).
	public:
		boost::polygon::voronoi_diagram<double> VD;

	};

}