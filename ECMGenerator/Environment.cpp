#include "Environment.h"
#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "ECMGenerator.h"

namespace ECM {

	void Environment::AddWalkableArea(std::vector<Segment> waEdges)
	{
		_walkableArea = waEdges;

		for (Segment s : waEdges) _environmentObstacleUnion.push_back(s);

		UpdateBbox(waEdges);
	}

	void Environment::AddObstacle(std::vector<Segment> obstacleEdges)
	{
		_obstacles.push_back(obstacleEdges);
		for (Segment s : obstacleEdges) _environmentObstacleUnion.push_back(s);

		UpdateBbox(obstacleEdges);
	}

	void Environment::ComputeECM()
	{		
		// for now just support generation of 1 ecm. However, an environment may contain various ECM graphs.
		m_EcmList.push_back(ECMGenerator::GenerateECM(*this));
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
			if (Utility::MathUtility::Contains(p, obs)) return true;
		}

		return false;
	}

	std::vector<Point> Environment::GetClosestObstaclePoints(const Point& location) const
	{
		std::vector<Point> result;
		float lowestSqrDistance = 20000000.0f; // fix this!!

		// todo: implement kd tree or better acceleration structure

		// TODO: very ugly code duplication! refactor!
		// Idea: maak virtuele obstacles naast de randen van de walkable area. dan behandel je deze randen gewoon als
		// obstakels, en hoef je niet allerlei edge cases met walkable area edges te doen.

		// TODO: BUG
		// > we loop through all segments, but this causes the closest obstacle point to be stored twice.

		// first test against walkable area boundaries
		using Utility::HALF_EPSILON;

		for (const Segment& seg : _walkableArea)
		{
			Point p = Utility::MathUtility::GetClosestPointOnSegment(location, seg);
			float sqrDist = Utility::MathUtility::SquareDistance(p, location);

			if (sqrDist < lowestSqrDistance)
			{
				result.clear();
				result.push_back(p);
				lowestSqrDistance = sqrDist;

				continue;
			}

			if (sqrDist > lowestSqrDistance - HALF_EPSILON && sqrDist < lowestSqrDistance + HALF_EPSILON)
			{
				result.push_back(p);
			}
		}

		// then against obstacle edges
		for (const auto& obs : _obstacles)
		{
			for (const Segment& seg : obs)
			{
				Point p = Utility::MathUtility::GetClosestPointOnSegment(location, seg);
				float sqrDist = Utility::MathUtility::SquareDistance(p, location);

				if (sqrDist < lowestSqrDistance)
				{
					result.clear();
					result.push_back(p);
					lowestSqrDistance = sqrDist;

					continue;
				}

				if (sqrDist > lowestSqrDistance - HALF_EPSILON && sqrDist < lowestSqrDistance + HALF_EPSILON)
				{
					result.push_back(p);
				}
			}
		}

		return result;
	}

	// for now just return the only ECM we have. However, this method can be expanded to be used with multiple ECM graphs.
	std::shared_ptr<ECM> Environment::QueryECM(Point position) const
	{
		return m_EcmList[0];
	}
}