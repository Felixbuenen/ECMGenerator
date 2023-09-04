#pragma once

#include <vector>
#include <memory>

//#include "ECM.h"


namespace ECM
{
	struct Point;
	struct Vec2;
	
	class Environment;
	class AStar;
	class ECMGraph;
	class ECM;

	namespace PathPlanning
	{
		struct Corridor
		{
			int numDisks;

			std::vector<Vec2> diskCenters;
			std::vector<float> diskRadii;
			std::vector<Vec2> leftBounds;
			std::vector<Vec2> rightBounds;
			// shrunk corridor bounds
			std::vector<Vec2> leftCorridorBounds; 
			std::vector<Vec2> rightCorridorBounds; 
		};

		typedef std::vector<Point> Path;

		// TODO:
		// > In a game, some segments of the path can be generated only after a certain event.
		// > For example, an agent could jump from 1 cliff edge to another. You want the user to be able
		//   to add these kinds of 'connector' events in the environment.
		// > Also interesting for formation  movement: a formation could maybe only fit through a gate if they shrink in.
		// > So, an idea is to return a path object that contains a list of points (the path) as well as a list of events.
		// > Maybe even better: just return a list of events. Each point is just a 'go to point' event. Once the event is completed
		//   you go and process the next event. 
		class ECMPathPlanner
		{
			// static class that uses an ECM object to plan paths

		public:
			ECMPathPlanner();
			~ECMPathPlanner();

			bool Initialize(ECMGraph& graph);

			// standard path query
			Path GetPath(const Environment& environment, Point start, Point goal, float clearance);

			// path query with a preferred clearance (and min/max clearance)
			// probably a good idea to create a class ECMVariableClearanceCostFunction, with subclasses that calculate the cost of
			//  a path given prefClear, minClear, maxClear and cost multiplier.
			// I think cost will be calculated per medial axis point, not per edge. If you return the path with clearances,
			//  you can decide in UE how you're going to dynamically change the formation along the path. This is important for e.g.
			//  when a corridor starts small and ends very wide (you don't want an unnecessarily small formation).
			//ECMPath* GetPath(/*ECM* ecm, point& start, point& goal, prefClear, minClear, maxClear, costFunc, costMultiplier*/) const;

		private:
			void RetractQueryPoints(const Point& start, const Point& goal, Point& outRetractedStart, Point& outRetractedGoal);

			void CreateCorridor(const std::vector<int>& maPath, Corridor& outCorridor, std::shared_ptr<ECM> ecm); // A* search on medial axis, returns set of edges
			void SmoothPath(/*path*/) const; // takes the medial axis path and creates and actual path to follow

			AStar* m_AStar;
		};
	}
}