#pragma once

#include <vector>
#include "ECMDataTypes.h"

namespace ECM {

	class ECMGraph;
	class ECMEdge;

	struct Segment;
	struct Point;

	// an ECM cell is defined by a boundary and the ECM edge it contains
	struct ECMCell
	{
		Segment boundary;
		ECMEdge* edge;
	};

	// stores the ECM cells and provides an API to perform operations (such as point location query).
	// TODO: 
	// > right now this is a very simple implementation, make it into a trapezoidal decomposition.
	class ECMCellCollection
	{
	public:
		void Construct(ECMGraph& graph);
		ECMCell* PointLocationQuery(ECMGraph& graph, const Point& location);

	private:
		std::vector<ECMCell> m_ECMCells; // todo: trapezoidal decomp
	};

}