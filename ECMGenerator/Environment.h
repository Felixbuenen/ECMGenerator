#pragma once

#include "ECMDataTypes.h"

#include <vector>
#include <memory>

namespace ECM {

	class ECMEdge;
	class ECM;

	class Environment
	{
		// an environment is the 2D "world" for which we generate the ECM.
		// An environment consists of:
		// > Walkable area
		// > A set of obstacles
		// > An ECM graph (aka navigation mesh)
		// 
		// This means that we initialize an Environment, and the environment HAS AN ECM.
		// The ECM can then be calculated usign the environment data.
		//
		// For now, we only support 2D environments (aka no multilayered environments)
		//
		//
		// Need to think about how we represent geometrical data. Probably best to do everything in Boost types. This
		//  prevents us from having to cast data. Besides that, we don't have to create our own points/segment/etc structs.
		// We can always refactor this later!

		// TODO:
		// > create walkable area and obstacle structs

	public:
		enum class TestEnvironment
		{
			CLASSIC
		};

		void AddWalkableArea(std::vector<Segment> waEdges); // for now just allow 1 walkable area
		void AddObstacle(std::vector<Segment> obstacleEdges);

		void ComputeECM();

		inline const std::vector<Segment>& GetWalkableArea() const { return _walkableArea; }
		inline const std::vector<std::vector<Segment>>& GetObstacles() const { return _obstacles; }
		inline BBOX GetBBOX() const { return _bbox; }
		std::vector<Point> GetClosestObstaclePoints(const Point& location) const;
		const std::vector<Segment>& GetEnvironmentObstacleUnion() const { return _environmentObstacleUnion; }
		std::shared_ptr<ECM> GetECM(int index = 0) { return m_EcmList[index]; }

		bool InsideObstacle(const Point& p) const;

	private:
		void UpdateBbox(const std::vector<Segment>& newEdges);

	private:
		BBOX _bbox;

		std::vector<Segment> _walkableArea;
		std::vector<std::vector<Segment>> _obstacles;
		std::vector<Segment> _environmentObstacleUnion;

		std::vector<std::shared_ptr<ECM>> m_EcmList;
	};

}