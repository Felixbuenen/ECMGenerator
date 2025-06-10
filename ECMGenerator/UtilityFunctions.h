#pragma once

#include "ECMDataTypes.h"

#include <cmath>
#include <vector>

namespace ECM {

	struct Point;
	struct Segment;
	struct Vec2;
	struct Obstacle;

	namespace Utility {
		class MathUtility
		{
		public:
			static float Distance(const Point& p1, const Point& p2);
			static float Distance(float x0, float y0, float x1, float y1);
			static float SquareDistance(const Point& p1, const Point& p2);
			static float SquareDistance(float x0, float y0, float x1, float y1);
			static float LineLeftDistance(const Point& v1, const Point& v2, const Point& p1);
			static bool Contains(const Point& p, const std::vector<Segment>& polygon);
			static bool Contains(const Point& p, const Obstacle& obstacle);
			static bool Intersect(const std::vector<Segment>& p1, const std::vector<Segment>& p2);
			static bool IsPointInQuadrilateral(const Point& point, const Point& A, const Point& B, const Point& C, const Point& D);
			static bool IsPointOnSegment(const Point& start, const Point& end, const Point& p);
			static float Dot(const Vec2& v1, const Vec2& v2);
			static float Dot(float x0, float y0, float x1, float y1);
			static float Determinant(const Vec2& v1, const Vec2& v2);
			static float Cross(const Vec2& v1, const Vec2& v2);
			static bool IsLeftOfSegment(const Segment& s, const Point& p);
			static bool IsLeftOfVector(const Vec2& base, const Vec2& vecToCheck);
			static float TriangleArea(const Point& p1, const Point& p2, const Point& p3);
			static Vec2 Right(const Vec2& v);
			static Vec2 Right(float x, float y);
			static Vec2 Left(const Vec2& v);
			static Vec2 Left(float x, float y);
			static Vec2 RotateVector(const Vec2& v, float rad);
			static float SquaredLength(const Vec2& v);
			static float SquaredLength(float x, float y);
			static float Length(const Vec2& v);
			static float Length(float x, float y);
			static Point GetClosestPointOnSegment(const Point& point, const Segment& segment);
			static Point GetClosestPointOnSegment(const Point& point, const Point& segPt1, const Point& segPt2);
			static Point GetClosestPointOnLine(const Point& linePoint, const Vec2& normalizedLineDir, const Point& referencePoint);
			static Point GetClosestPointOnLineThroughOrigin(const Vec2& normalizedLineDir, const Point& referencePoint);
			static bool GetRayToLineSegmentIntersection(const Point& rayOrigin, const Vec2& rayDirection, const Point& point1, const Point& point2, Point& outPoint, float& outDist);

			static const float PI;
		};
	}
}