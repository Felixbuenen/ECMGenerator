#pragma once

#include "ECMDataTypes.h"

#include <vector>
#include <memory>

namespace ECM {

	class ECMEdge;
	class ECMCell;
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

	public:
		enum class TestEnvironment
		{
			CLASSIC,
			BIG,
			NARROW,
			LINES,
			EMPTY
		};

		void Initialize(TestEnvironment type);
		void AddWalkableArea(std::vector<Segment> waEdges); // for now just allow 1 walkable area
		void AddObstacle(std::vector<Segment> obstacleEdges);

		void ComputeECM();

		inline const std::vector<Segment>& GetWalkableArea() const { return m_WalkableArea; }
		inline const std::vector<Obstacle>& GetObstacles() const { return m_Obstacles; }
		inline BBOX GetBBOX() const { return m_Bbox; }
		inline const std::vector<Segment>& GetEnvironmentObstacleUnion() const { return m_EnvironmentObstacleUnion; }
		inline std::shared_ptr<ECM> GetECM(int index = 0) { return m_EcmList[index]; }

		std::shared_ptr<ECM> QueryECM(Point position) const;

		bool InsideObstacle(const Point& p) const;

	private:
		void UpdateBbox(const std::vector<Segment>& newEdges);

		void CreateTestEnvironment_U();

	private:
		BBOX m_Bbox;

		std::vector<Segment> m_WalkableArea;
		std::vector<std::vector<Segment>> m_ObstaclesDeprecated;
		std::vector<Obstacle> m_Obstacles;
		std::vector<Segment> m_EnvironmentObstacleUnion;

		std::vector<std::shared_ptr<ECM>> m_EcmList;
	};

}