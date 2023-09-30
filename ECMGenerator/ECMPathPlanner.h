#pragma once

#include <vector>
#include <memory>

//#include "ECM.h"


namespace ECM
{
	struct Point;
	struct Vec2;
	struct Segment;
	struct ECMHalfEdge;

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
			ECMPathPlanner();
			~ECMPathPlanner();

			bool Initialize(ECMGraph& graph);

			// standard path query
			bool GetPath(const Environment& environment, Point start, Point goal, float clearance, Corridor& outCorridor, std::vector<Segment>& outPortals, Path& outPath);

		private:
			void RetractQueryPoints(const Point& start, const Point& goal, Point& outRetractedStart, Point& outRetractedGoal);

			void CreateCorridor(const std::vector<ECMHalfEdge*>& maPath, Corridor& outCorridor, std::shared_ptr<ECM> ecm); // A* search on medial axis, returns set of edges
			void ShrinkCorridor(Corridor& corridor, float clearance);
			void TriangulateCorridor(const Point& start, const Point& goal, const Corridor& corridor, std::vector<Segment>& outPortals, float clearance);
			void FindFirstAndLastPortal(const Point& start, const Point& goal, const std::vector<Segment>& portals, int& outFirst, int& outLast);
			void SampleCorridorArc(const Point& p1, const Point& p2, const Point& o1, const Point& o2, const Point& c, float radius, bool leftArc, std::vector<Segment>& portals);
			void FindFirstAndLastPortal(const std::vector<Segment>& portals, const Point& start, const Point& goal, int& outFirst, int& outLast);
			void Funnel(const std::vector<Segment>& portals, const Point& start, const Point& goal, std::vector<Point>& outShortestPath);
			void SmoothPath(/*path*/) const; // takes the medial axis path and creates and actual path to follow

			AStar* m_AStar;
		};
	}
}