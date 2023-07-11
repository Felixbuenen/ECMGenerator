#pragma once

class ECMPath;

class ECMPathPlanner
{
	// static class that uses an ECM object to plan paths

public:
	// standard path query
	ECMPath* GetPath(/*ECM* ecm, point& start, point& goal, float clearance*/) const;

	// path query with a preferred clearance (and min/max clearance)
	// probably a good idea to create a class ECMVariableClearanceCostFunction, with subclasses that calculate the cost of
	//  a path given prefClear, minClear, maxClear and cost multiplier.
	// I think cost will be calculated per medial axis point, not per edge. If you return the path with clearances,
	//  you can decide in UE how you're going to dynamically change the formation along the path. This is important for e.g.
	//  when a corridor starts small and ends very wide (you don't want an unnecessarily small formation).
	//ECMPath* GetPath(/*ECM* ecm, point& start, point& goal, prefClear, minClear, maxClear, costFunc, costMultiplier*/) const;

private:
	void GetMedialAxisPath() const; // A* search on medial axis, returns set of edges
	void SmoothPath(/*path*/) const; // takes the medial axis path and creates and actual path to follow
};

