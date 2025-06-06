#pragma once

#include <vector>
#include <memory>

#include "AStar.h"


namespace ECM
{
	struct Point;
	struct Vec2;
	struct Segment;
	struct ECMHalfEdge;
	struct ECMEdge;

	class Environment;
	class AStar;
	class ECMGraph;
	class ECM;

	namespace PathPlanning
	{
		enum CorridorBoundCurve
		{
			LINEAR,
			LEFT_ARC,
			RIGHT_ARC
		};

		struct Corridor
		{
			std::vector<Point> diskCenters;
			std::vector<float> diskRadii;
			std::vector<Point> leftBounds;
			std::vector<Point> rightBounds;
			// shrunk corridor bounds
			std::vector<Point> leftCorridorBounds;
			std::vector<Point> rightCorridorBounds;

			std::vector<CorridorBoundCurve> curveTypes;
		};

		typedef std::vector<Point> Path;

		class ECMPathPlanner
		{
			// static class that uses an ECM object to plan paths

		public:
			ECMPathPlanner(ECMGraph* graph);
			~ECMPathPlanner();

			// standard path query
			bool FindPath(const Environment& environment, Point start, Point goal, float clearance, float preferredClearance, Corridor& outCorridor, std::vector<Segment>& outPortals, Path& outPath);
			void HandleECMUpdate() { m_AStar.HandleECMUpdate(); }

		private:
			bool ValidStartGoalLocation(const Point& start, const Point& goal, const ECMEdge& startEdge, const ECMEdge& goalEdge, float clearance) const;

			void CreateCorridor(const std::vector<const ECMHalfEdge*>& maPath, Corridor& outCorridor, ECM* ecm); // A* search on medial axis, returns set of edges
			void ShrinkCorridor(Corridor& corridor, float clearance);
			void TriangulateCorridor(const Point& start, const Point& goal, const Corridor& corridor, std::vector<Segment>& outPortals, float clearance);
			void SampleCorridorArc(const Point& p1, const Point& p2, const Point& o1, const Point& o2, const Point& c, float radius, bool leftArc, std::vector<Segment>& portals);
			void FitPortalRange(std::vector<Segment>& portals, const Point& start, const Point& goal);
			void Funnel(const std::vector<Segment>& portals, const Point& start, const Point& goal, std::vector<Point>& outShortestPath);

			AStar m_AStar;
		};
	}
}