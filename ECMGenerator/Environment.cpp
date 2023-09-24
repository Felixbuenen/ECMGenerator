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

	// for now just return the only ECM we have. However, this method can be expanded to be used with multiple ECM graphs.
	std::shared_ptr<ECM> Environment::QueryECM(Point position) const
	{
		return m_EcmList[0];
	}
}