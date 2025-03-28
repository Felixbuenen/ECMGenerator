#pragma once

#include <vector>
#include "ECMDataTypes.h"

namespace ECM {

	class ECMGraph;
	class ECMEdge;
	class TrapezoidalDecomposition;

	struct Segment;
	struct Point;

	// an ECM cell is defined by a boundary and the ECM edge it contains
	struct ECMCell
	{
		Segment boundary;
		ECMEdge* edge;
	};

	enum class ECMCellCollectionType
	{
		LINEAR,
		TDC
	};

	// stores the ECM cells and provides an API to perform operations (such as point location query).
	// TODO: 
	// > right now this is a very simple implementation, make it into a trapezoidal decomposition.
	class ECMCellCollection
	{

	public:
		void Construct(ECMGraph& graph, ECMCellCollectionType type);
		void Clear();

		const ECMCell* PointLocationQueryLinear(const ECMGraph& graph, const Point& location) const;

	private:
		std::vector<ECMCell> m_ECMCells;
		TrapezoidalDecomposition* m_ECMCellDecomp;
	};

}