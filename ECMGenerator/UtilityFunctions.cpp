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