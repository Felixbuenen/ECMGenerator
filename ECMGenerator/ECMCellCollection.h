#pragma once

#include <vector>
#include "ECMDataTypes.h"

namespace ECM {

	class ECMGraph;
	class ECMEdge;
	class TrapezoidalDecomposition;
	class Environment;

	struct Segment;
	struct Point;

	// an ECM cell is defined by a boundary and the ECM edge it contains
	struct ECMCell
	{
		int idx;
		Segment boundary;
		ECMEdge* edge;
	};

	enum class ECMCellCollectionType
	{
		LINEAR,
		GRID,
		TDC,
	};

	// stores the ECM cells and provides an API to perform operations (such as point location query).
	class ECMCellCollection
	{

	public:
		void ConstructDefault(ECMGraph& graph);
		void ConstructWithGrid(ECMGraph& graph, const Environment& env, float cellSize = 100.0f);
		void Clear();

		const ECMCell* PointLocationQueryLinear(const ECMGraph& graph, const Point& location, int hintIdx = -1) const;
		inline const ECMCell* GetCell(int idx) { return &m_ECMCells[idx]; }

	private:
		void AddCellToGrid(ECMGraph& graph, const Point& gridBBOXMin, const ECMCell& cell);

		std::vector<ECMCell> m_ECMCells;
		float m_GridCellSize;
		std::vector<uint32_t> m_GridLookup;
		TrapezoidalDecomposition* m_ECMCellDecomp;
	};

}