#include "UtilityFunctions.h"
#include "ECMDataTypes.h"

#include <cmath>

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

bool MathUtility::Contains(const Point& p, const std::vector<Segment>& polygon)
{
	// ray cast algorithm

	bool inside = false;

	for(const Segment& s : polygon)
	{
		// check if point equal to segment endpoint (in which case we don't want it to be considered "contained"
		// this check is enough for the purpose of this demo: we don't have a case where we need to check if a point
		// is on a line segment.
		if (p.x == s.p0.x && p.y == s.p0.y) return false;
		if (p.x == s.p1.x && p.y == s.p1.y) return false;

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

bool MathUtility::Contains(const Segment& p, const std::vector<Segment>& polygon)
{
	// TODO: implement. for now we don't need it
	return false;
}

float MathUtility::Dot(const Vec2& v1, const Vec2& v2)
{
	return v1.x * v2.x + v1.y * v2.y;
}

float MathUtility::Dot(float x0, float y0, float x1, float y1)
{
	return x0 * x1 + y0 * y1;
}

// TODO: maybe it's nicer to include these type-specific functions as static type methods (e.g. Vec2::Right(..)).
Vec2 MathUtility::Right(const Vec2& v)
{
	Vec2 result;
	result.x = v.y;
	result.y = -v.x;

	return result;
}

Vec2 MathUtility::Right(float x, float y)
{
	Vec2 result;
	result.x = y;
	result.y = -x;

	return result;
}

float MathUtility::SquaredLength(const Vec2& v)
{
	return v.x * v.x + v.y * v.y;
}

float MathUtility::SquaredLength(float x, float y)
{
	return x * x + y * y;
}

Point MathUtility::GetClosestPointOnSegment(const Point& point, const Segment& segment)
{
	// dot product to find closest point on a segment to another point

	Point segmentVec = segment.p1 - segment.p0;
	Point pToSeg = point - segment.p0;
	float tSquared = MathUtility::SquareDistance(segment.p0, segment.p1);
	float dotProd = (pToSeg.x * segmentVec.x + pToSeg.y * segmentVec.y) / tSquared;

	if (dotProd > 1.0) dotProd = 1.0f;
	if (dotProd < 0.0) dotProd = 0.0f;

	Point posOnSegment(segment.p0.x + dotProd * segmentVec.x, segment.p0.y + dotProd * segmentVec.y); // todo: implement t * Point operator

	return posOnSegment;
}
