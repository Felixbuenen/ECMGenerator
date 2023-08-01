#pragma once

#include <vector>

class ECMGraph;

struct Segment;
struct Point;

// an ECM cell is defined by a boundary and the ECM edge it contains
struct ECMCell
{
	std::vector<Segment> boundary;
	int ecmEdge;
};

// stores the ECM cells and provides an API to perform operations (such as point location query).
// TODO: 
// > right now this is a very simple implementation, make it into a trapezoidal decomposition.
class ECMCellCollection
{
public:
	void Construct(const ECMGraph& graph);
	const ECMCell& PointLocationQuery(const Point& location) const;

private:
	std::vector<ECMCell> m_ECMCells; // todo: trapezoidal decomp
};