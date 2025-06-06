#include "UtilityFunctions.h"
#include "ECMDataTypes.h"

#include <cmath>

// TODO:
// remove class definition. simplify by making defining methods without class.


namespace ECM {
	namespace Utility {

		const float MathUtility::PI = 3.141593f;

		float MathUtility::Distance(const Point& p1, const Point& p2)
		{
			Point diff;
			diff.x = p2.x - p1.x;
			diff.y = p2.y - p1.y;

			return sqrtf(diff.x * diff.x + diff.y * diff.y);
		}

		float MathUtility::Distance(float x0, float y0, float x1, float y1)
		{
			float diffX = x0 - x1;
			float diffY = y0 - y1;

			return sqrtf(diffX * diffX + diffY * diffY);
		}

		float MathUtility::SquareDistance(const Point& p1, const Point& p2)
		{
			Point diff;
			diff.x = p2.x - p1.x;
			diff.y = p2.y - p1.y;

			return diff.x * diff.x + diff.y * diff.y;
		}

		float MathUtility::SquareDistance(float x0, float y0, float x1, float y1)
		{
			float diffX = x0 - x1;
			float diffY = y0 - y1;

			return diffX * diffX + diffY * diffY;
		}

		float MathUtility::LineLeftDistance(const Point& v1, const Point& v2, const Point& p1)
		{
			return MathUtility::Determinant(v1 - p1, v2 - v1);
		}

		bool MathUtility::Contains(const Point& p, const std::vector<Segment>& polygon)
		{
			// ray cast algorithm

			bool inside = false;

			for (const Segment& s : polygon)
			{
				// check if point equal to segment endpoint (in which case we don't want it to be considered "contained"
				// this check is enough for the purpose of this demo: we don't have a case where we need to check if a point
				// is on a line segment.
				if (p.Approximate(s.p0)) return false;
				if (p.Approximate(s.p1)) return false;

				if (p.y > fmin(s.p0.y, s.p1.y))
				{
					if (p.y < fmax(s.p0.y, s.p1.y))
					{
						if (p.x < fmax(s.p0.x, s.p1.x))
						{
							float x_intersection = (p.y - s.p0.y) * (s.p1.x - s.p0.x) / (s.p1.y - s.p0.y) + s.p0.x;

							if (s.p0.x == s.p1.x || p.x < x_intersection)
							{
								inside = !inside;
							}
						}
					}
				}
			}

			return inside;
		}

		bool MathUtility::Contains(const Point& p, const Obstacle& obstacle)
		{
			// ray cast algorithm

			bool inside = false;

			const auto& verts = obstacle.verts;

			for (const ObstacleVertex* v : verts)
			{
				const Point& p0 = v->p;
				if (v->nextObstacle == nullptr) return false;

				const Point& p1 = v->nextObstacle->p;

				// check if point equal to segment endpoint (in which case we don't want it to be considered "contained"
				// this check is enough for the purpose of this demo: we don't have a case where we need to check if a point
				// is on a line segment.
				if (p.Approximate(p0)) return false;
				if (p.Approximate(p1)) return false;

				if (p.y > fmin(p0.y, p1.y))
				{
					if (p.y < fmax(p0.y, p1.y))
					{
						if (p.x < fmax(p0.x, p1.x))
						{
							float x_intersection = (p.y - p0.y) * (p1.x - p0.x) / (p1.y - p0.y) + p0.x;

							if (p0.x == p1.x || p.x < x_intersection)
							{
								inside = !inside;
							}
						}
					}
				}
			}

			return inside;
		}


		bool MathUtility::IsPointInQuadrilateral(const Point& point, const Point& A, const Point& B, const Point& C, const Point& D) {
			// Define the four edges of the quadrilateral
			Point edge1 = B - A;
			Point edge2 = C - B;
			Point edge3 = D - C;
			Point edge4 = A - D;

			// Calculate vectors from one vertex to the test point
			Point vec1 = point - A;
			Point vec2 = point - B;
			Point vec3 = point - C;
			Point vec4 = point - D;

			// Calculate cross products between vectors
			double crossProduct1 = Cross(edge1, vec1);
			double crossProduct2 = Cross(edge2, vec2);
			double crossProduct3 = Cross(edge3, vec3);
			double crossProduct4 = Cross(edge4, vec4);

			// Check if all cross products have the same sign
			if (crossProduct1 * crossProduct2 > 0 &&
				crossProduct2 * crossProduct3 > 0 &&
				crossProduct3 * crossProduct4 > 0) {
				return true; // Point is within the quadrilateral boundaries
			}
			else {
				return false; // Point is outside the quadrilateral boundaries
			}
		}

		bool MathUtility::IsPointOnSegment(const Point& start, const Point& end, const Point& p)
		{
			Vec2 seg = end - start;
			Vec2 toPoint = p - start;

			float t = Dot(seg, toPoint);
			float segLengthSq = seg.LengthSquared();

			return t >= 0 && t <= segLengthSq;
		}


		float MathUtility::Dot(const Vec2& v1, const Vec2& v2)
		{
			return v1.x * v2.x + v1.y * v2.y;
		}

		float MathUtility::Dot(float x0, float y0, float x1, float y1)
		{
			return x0 * x1 + y0 * y1;
		}

		float MathUtility::Determinant(const Vec2 & v1, const Vec2 & v2)
		{
			return v1.x * v2.y - v1.y * v2.x;
		}


		float MathUtility::Cross(const Vec2& v1, const Vec2& v2)
		{
			return v1.x * v2.y - v1.y * v2.x;
		}

		bool MathUtility::IsLeftOfSegment(const Segment& s, const Point& p)
		{
			return (s.p1.x - s.p0.x) * (p.y - s.p0.y) - (s.p1.y - s.p0.y) * (p.x - s.p0.x) > 0;
		}

		bool MathUtility::IsLeftOfVector(const Vec2& base, const Vec2& vecToCheck)
		{
			return base.x * vecToCheck.y - base.y * vecToCheck.x > 0;
		}

		float MathUtility::TriangleArea(const Point& p1, const Point& p2, const Point& p3)
		{
			float ax = p2.x - p1.x;
			float ay = p2.y - p1.y;
			float bx = p3.x - p1.x;
			float by = p3.y - p1.y;
			return bx * ay - ax * by;
		}

		// TODO: maybe it's nicer to include these type-specific functions as static type methods (e.g. Vec2::Right(..)).
		Vec2 MathUtility::Right(const Vec2& v)
		{
			return Vec2(v.y, -v.x);
		}

		Vec2 MathUtility::Right(float x, float y)
		{
			return Vec2(y, -x);
		}

		Vec2 MathUtility::Left(const Vec2& v)
		{
			return Vec2(-v.y, v.x);
		}

		Vec2 MathUtility::Left(float x, float y)
		{
			return Vec2(-y, x);
		}

		Vec2 MathUtility::RotateVector(const Vec2& v, float rad)
		{
			float cs = cos(rad);
			float sn = sin(rad);

			float px = v.x * cs - v.y * sn;
			float py = v.x * sn + v.y * cs;

			return Vec2(px, py);
		}


		float MathUtility::SquaredLength(const Vec2& v)
		{
			return v.x * v.x + v.y * v.y;
		}

		float MathUtility::SquaredLength(float x, float y)
		{
			return x * x + y * y;
		}

		float MathUtility::Length(const Vec2& v)
		{
			return sqrt(v.x * v.x + v.y * v.y);
		}

		float MathUtility::Length(float x, float y)
		{
			return sqrt(x * x + y * y);
		}


		Point MathUtility::GetClosestPointOnSegment(const Point& point, const Segment& segment)
		{
			// dot product to find closest point on a segment to another point
			if (segment.p0.Approximate(segment.p1))
			{
				return segment.p0;
			}

			Point segmentVec = segment.p1 - segment.p0;
			Point pToSeg = point - segment.p0;
			float tSquared = MathUtility::SquareDistance(segment.p0, segment.p1);
			float dotProd = (pToSeg.x * segmentVec.x + pToSeg.y * segmentVec.y) / tSquared;

			if (dotProd > 1.0) dotProd = 1.0f;
			if (dotProd < 0.0) dotProd = 0.0f;

			Point posOnSegment(segment.p0.x + dotProd * segmentVec.x, segment.p0.y + dotProd * segmentVec.y); // todo: implement t * Point operator

			return posOnSegment;
		}

		Point MathUtility::GetClosestPointOnSegment(const Point& point, const Point& segPt1, const Point& segPt2)
		{
			// dot product to find closest point on a segment to another point
			if (segPt1.Approximate(segPt2))
			{
				return segPt1;
			}

			Point segmentVec = segPt2 - segPt1;
			Point pToSeg = point - segPt1;
			float tSquared = MathUtility::SquareDistance(segPt1, segPt2);
			float dotProd = (pToSeg.x * segmentVec.x + pToSeg.y * segmentVec.y) / tSquared;

			if (dotProd > 1.0) dotProd = 1.0f;
			if (dotProd < 0.0) dotProd = 0.0f;

			Point posOnSegment(segPt1.x + dotProd * segmentVec.x, segPt1.y + dotProd * segmentVec.y); // todo: implement t * Point operator

			return posOnSegment;
		}

		Point MathUtility::GetClosestPointOnLine(const Point& linePoint, const Vec2& normalizedLineDir, const Point& referencePoint)
		{
			Vec2 pointToRef = Vec2(referencePoint.x - linePoint.x, referencePoint.y - linePoint.y);
			float lenToPoint = Dot(pointToRef, normalizedLineDir);

			return (linePoint + normalizedLineDir * lenToPoint);
		}

		Point MathUtility::GetClosestPointOnLineThroughOrigin(const Vec2& normalizedLineDir, const Point& referencePoint)
		{
			float lenToPoint = Dot(Vec2(referencePoint), normalizedLineDir);
			return Point(normalizedLineDir * lenToPoint);
		}


		bool MathUtility::GetRayToLineSegmentIntersection(const Point& rayOrigin, const Vec2& rayDirection, const Point& point1, const Point& point2, Point& outPoint, float& outDist)
		{
			Vec2 v1 = rayOrigin - point1;
			Vec2 v2 = point2 - point1;
			Vec2 v3(-rayDirection.y, rayDirection.x);

			float dot = MathUtility::Dot(v2, v3);
			if (abs(dot) < 0.000001)
			{
				return false;
			}
			
			float t1 = MathUtility::Cross(v2, v1) / dot;
			float t2 = MathUtility::Dot(v1, v3) / dot;


			if (t1 >= 0.0 && (t2 >= 0.0 && t2 <= 1.0))
			{
				outDist = t1;
				outPoint.x = rayOrigin.x + rayDirection.x * t1;
				outPoint.y = rayOrigin.y + rayDirection.y * t1;

				return true;
			}

			return false;
		}


	}
}