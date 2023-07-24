#pragma once

#include "boost/polygon/voronoi.hpp"

struct Point {
	float x;
	float y;
	Point(float x, float y) : x(x), y(y) {}
	Point() { x = 0; y = 0; }

	Point operator -(const Point& a) const {
		return Point(x - a.x, y - a.y);
	}
	Point operator =(const Point& a) const {
		return Point(a.x, a.y);
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
		min.x = 20000; // ---- UUUUUUGLY. make a define of it ----------
		min.y= 20000;
		max.x = -200000;
		max.y = -200000;
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

