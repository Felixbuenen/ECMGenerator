#include "Environment.h"
#include "UtilityFunctions.h"

void Environment::AddWalkableArea(std::vector<Segment> waEdges)
{
	_walkableArea = waEdges;
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
