#pragma once

#include "ECMDataTypes.h"

#include <vector>
#include <memory>

namespace ECM {

	class ECMEdge;
	class ECMCell;
	class ECM;

	namespace WindowApplication {
		class Application;
	}

	class Environment
	{
		// an environment is the 2D "world" for which we generate the ECM.
		// An environment consists of:
		// > Walkable area
		// > A set of obstacles
		// > An ECM graph (aka navigation mesh)
		// 
		// This means that we initialize an Environment, and the environment HAS AN ECM.
		// The ECM can then be calculated using the environment data.
		//
		// For now, we only support 2D environments (aka no multilayered environments)

	public:
		enum class TestEnvironment
		{
			CLASSIC,
			BIG,
			NARROW,
			LINES,
			SQUARE,
			EMPTY,
			DEBUG1
		};

		~Environment();

		void Initialize(TestEnvironment type);
		void AddWalkableArea(std::vector<Segment> waEdges); // for now just allow 1 walkable area
		void AddObstacle(const std::vector<Point>& obstacleVerts, bool updateECM = false);
		void SetDirty(bool dirty = true) { m_Dirty = dirty; }

		void ComputeECM();
		void UpdateECM();

		inline const std::vector<Segment>& GetWalkableArea() const { return m_WalkableArea; }
		inline const std::vector<Obstacle>& GetObstacles() const { return m_Obstacles; }
		inline BBOX GetBBOX() const { return m_Bbox; }
		inline const std::vector<Segment>& GetEnvironmentObstacleUnion() const { return m_EnvironmentObstacleUnion; }
		inline ECM* GetECM(int index = 0) { return m_Ecms[index]; }
		inline bool GetIsEnvironmentDirty() { return m_Dirty; }

		ECM* QueryECM(Point position) const;

		bool InsideObstacle(const Point& p) const;

	private:
		void UpdateBbox(const std::vector<Segment>& newEdges);

	private:
		bool m_Dirty;

		BBOX m_Bbox;
		std::vector<Segment> m_WalkableArea;

		// TODO: dynamic vector of obstacles invalidates obstacle references in that array. 
		//       Instead of "Obstacle" being a vertex, create an "Obstacle" class with a vector of
		//		 "ObstacleVertex" objects/structs. This vector does not change, so reference are possible.
		//       The dynamic vector will contain Obstacle objects.
		//       If this doesn't work, we must dynamically allocate ObstVerts objects and store the pointers.
		std::vector<Obstacle> m_Obstacles;
		std::vector<int> m_ObstacleIndices; // TODO: make obsolete
		std::vector<Segment> m_EnvironmentObstacleUnion;

		std::vector<ECM*> m_Ecms;
		//std::vector<std::shared_ptr<ECM>> m_EcmList;
	};

}