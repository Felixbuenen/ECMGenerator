#include "Environment.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"

void Environment::AddWalkableArea(std::vector<Segment> waEdges)
{
	_walkableArea = waEdges;

	// the edges of the walkable area should be considered an 'object'
	//AddObstacle(waEdges);

	UpdateBbox(waEdges);
}

void Environment::AddObstacle(std::vector<Segment> obstacleEdges)
{
	_obstacles.push_back(obstacleEdges);
	UpdateBbox(obstacleEdges);
}

void Environment::UpdateBbox(const std::vector<Segment>& newEdges)
{
	for (const Segment& edge : newEdges)
	{
		// update min values
		if (edge.p0.x < _bbox.min.x) _bbox.min.x = edge.p0.x;
		if (edge.p0.y < _bbox.min.y) _bbox.min.y = edge.p0.y;
		if (edge.p1.x < _bbox.min.x) _bbox.min.x = edge.p1.x;
		if (edge.p1.y < _bbox.min.y) _bbox.min.y = edge.p1.y;

		// update max values
		if (edge.p0.x > _bbox.max.x) _bbox.max.x = edge.p0.x;
		if (edge.p0.y > _bbox.max.y) _bbox.max.y = edge.p0.y;
		if (edge.p1.x > _bbox.max.x) _bbox.max.x = edge.p1.x;
		if (edge.p1.y > _bbox.max.y) _bbox.max.y = edge.p1.y;
	}
}

bool Environment::InsideObstacle(const Point& p) const
{
	// todo: something clever with kd-tree

	for (const auto& obs : _obstacles)
	{
		if (MathUtility::Contains(p, obs)) return true;
	}

	return false;
}

// todo: refactor, maybe we can make the method a little more generic (less ecm-specific)
void Environment::GetClosestObstaclePointsToEdge(const Point& edge_v1, const Point& edge_v2, Point& out_Left0, Point& out_Left1, Point& out_Right0, Point& out_Right1) const
{
	Point edgeDir = edge_v2 - edge_v1; // todo: make 'vector2' type.

	// idea: probably fastest to calculate all closest points for an ECMVertex.
	// Then, to get closest left/right, you:
	// 1. take per vertex de list of closest points
	// 2. use the dot product to determine which point is left/right, and which point should be disregarded.
	//    My hypothesis is that if dot < 0 or dot > 1 points should be disregarded
}

Point Environment::GetClosestPointOnSegment(const Point& point, const Segment& segment) const
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

std::vector<Point> Environment::GetClosestObstaclePoints(const Point& location) const
{
	std::vector<Point> result;
	float lowestSqrDistance = 20000000.0f; // fix this!!

	// todo: implement kd tree or better acceleration structure

	// TODO: very ugly code duplication! refactor!

	// first test against walkable area boundaries
	for (const Segment& seg : _walkableArea)
	{
		Point p = GetClosestPointOnSegment(location, seg);
		float sqrDist = MathUtility::SquareDistance(p, location);

		if (sqrDist < lowestSqrDistance)
		{
			result.clear();
			result.push_back(p);
			lowestSqrDistance = sqrDist;

			continue;
		}

		if (sqrDist > lowestSqrDistance - ECM_HALF_EPSILON && sqrDist < lowestSqrDistance + ECM_HALF_EPSILON)
		{
			result.push_back(p);
		}
	}

	// then against obstacle edges
	for (const auto& obs : _obstacles)
	{
		for (const Segment& seg : obs)
		{
			Point p = GetClosestPointOnSegment(location, seg);
			float sqrDist = MathUtility::SquareDistance(p, location);

			if (sqrDist < lowestSqrDistance)
			{
				result.clear();
				result.push_back(p);
				lowestSqrDistance = sqrDist;

				continue;
			}

			if (sqrDist > lowestSqrDistance - ECM_HALF_EPSILON && sqrDist < lowestSqrDistance + ECM_HALF_EPSILON)
			{
				result.push_back(p);
			}
		}
	}

	printf("number of closest vertices: %d\n", result.size());

	return result;
}
