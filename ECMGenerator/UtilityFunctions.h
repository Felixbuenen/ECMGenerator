#pragma once

#include <cmath>
#include <vector>

struct Point;
struct Segment;
struct Vec2;

class MathUtility
{
public:
	static float Distance(const Point& p1, const Point& p2);
	static float Distance(float x0, float y0, float x1, float y1);
	static float SquareDistance(const Point& p1, const Point& p2);
	static float SquareDistance(float x0, float y0, float x1, float y1);
	static bool Contains(const Point& p, const std::vector<Segment>& polygon);
	static bool Contains(const Segment& p, const std::vector<Segment>& polygon);
	static float Dot(const Vec2& v1, const Vec2& v2);
	static float Dot(float x0, float y0, float x1, float y1);
	static Vec2 Right(const Vec2& v);
	static Vec2 Right(float x, float y);
	static float SquaredLength(const Vec2& v);
	static float SquaredLength(float x, float y);
	static Point GetClosestPointOnSegment(const Point& point, const Segment& segment);

};