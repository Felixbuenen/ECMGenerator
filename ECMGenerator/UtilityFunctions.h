#pragma once

#include <cmath>
#include <vector>

struct Point;
struct Segment;

class MathUtility
{
public:
	static float Distance(const Point& p1, const Point& p2);
	static float Distance(float x0, float y0, float x1, float y1);
	static bool Contains(const Point& p, const std::vector<Segment>& polygon);
	static bool Contains(const Segment& p, const std::vector<Segment>& polygon);
};