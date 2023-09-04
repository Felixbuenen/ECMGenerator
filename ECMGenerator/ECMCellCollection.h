#pragma once

#include <vector>

namespace ECM {

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
		void Construct(ECMGraph& graph);
		ECMCell* PointLocationQuery(const Point& location);

	private:
		std::vector<ECMCell> m_ECMCells; // todo: trapezoidal decomp
	};

}