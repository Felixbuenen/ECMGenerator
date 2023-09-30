#pragma once

#include <cmath>
#include <vector>

namespace ECM {

	struct Point;
	struct Segment;
	struct Vec2;

	namespace Utility {
		class MathUtility
		{
		public:
			static float Distance(const Point& p1, const Point& p2);
			static float Distance(float x0, float y0, float x1, float y1);
			static float SquareDistance(const Point& p1, const Point& p2);
			static float SquareDistance(float x0, float y0, float x1, float y1);
			static bool Contains(const Point& p, const std::vector<Segment>& polygon);
			static float Dot(const Vec2& v1, const Vec2& v2);
			static float Dot(float x0, float y0, float x1, float y1);
			static float Cross(const Vec2& v1, const Vec2& v2);
			static bool IsLeftOfSegment(const Segment& s, const Point& p);
			static bool IsLeftOfVector(const Vec2& base, const Vec2& vecToCheck);
			static float TriangleArea(const Point& p1, const Point& p2, const Point& p3);
			static Vec2 Right(const Vec2& v);
			static Vec2 Right(float x, float y);
			static float SquaredLength(const Vec2& v);
			static float SquaredLength(float x, float y);
			static float Length(const Vec2& v);
			static float Length(float x, float y);
			static Point GetClosestPointOnSegment(const Point& point, const Segment& segment);
			static bool GetRayToLineSegmentIntersection(const Point& rayOrigin, const Vec2& rayDirection, const Point& point1, const Point& point2, Point& outPoint, float& outDist);

			static const float PI;
		};
	}
}