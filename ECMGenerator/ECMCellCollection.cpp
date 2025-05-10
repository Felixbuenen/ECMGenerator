#include "ECMCellCollection.h"

#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "TrapezoidalDecomposition.h"
#include "Environment.h"
#include "Datastructures.h"

namespace ECM {

	void ECMCellCollection::ConstructDefault(ECMGraph& graph)
	{
		int idx = 0;
		for (ECMEdge& edge : graph.GetEdges())
		{
			ECMCell cellLeft;
			ECMCell cellRight;

			const ECMHalfEdge& edge0 = edge.half_edges[0];
			const ECMHalfEdge& edge1 = edge.half_edges[1];

			const ECMVertex* v0 = graph.GetVertex(edge.half_edges[1].v_target_idx);
			const ECMVertex* v1 = graph.GetVertex(edge.half_edges[0].v_target_idx);

			Point p0 = v0->position;
			Point p1 = v1->position;

			Point p0_left = edge0.closest_left;
			Point p1_left = edge1.closest_right;
			Point p0_right = edge0.closest_right;
			Point p1_right = edge1.closest_left;

			cellLeft.boundary.p0 = p0_left;
			cellLeft.boundary.p1 = p1_left;
			cellRight.boundary.p0 = p0_right;
			cellRight.boundary.p1 = p1_right;
			cellLeft.edge = &edge;
			cellRight.edge = &edge;

			cellLeft.idx = idx;
			idx++;
			cellRight.idx = idx;
			idx++;

			m_ECMCells.push_back(cellLeft);
			m_ECMCells.push_back(cellRight);
		}
	}

	void ECMCellCollection::ConstructWithGrid(ECMGraph& graph, const Environment& env, float cellSize)
	{
		m_GridCellSize = cellSize;

		// store ECM cell references in grid structure for fast lookup
		Vec2 size = env.GetBBOX().max - env.GetBBOX().min;
		int numCols = std::ceil(size.x / cellSize);
		int numRows = std::ceil(size.y / cellSize);

		m_GridLookup.resize(numCols * numRows);

		int idx = 0;
		// calculate ECM cells
		for (ECMEdge& edge : graph.GetEdges())
		{
			ECMCell cellLeft;
			ECMCell cellRight;

			const ECMHalfEdge& edge0 = edge.half_edges[0];
			const ECMHalfEdge& edge1 = edge.half_edges[1];

			const ECMVertex* v0 = graph.GetVertex(edge.half_edges[1].v_target_idx);
			const ECMVertex* v1 = graph.GetVertex(edge.half_edges[0].v_target_idx);

			Point p0 = v0->position;
			Point p1 = v1->position;

			Point p0_left = edge0.closest_left;
			Point p1_left = edge1.closest_right;
			Point p0_right = edge0.closest_right;
			Point p1_right = edge1.closest_left;

			cellLeft.boundary.p0 = p0_left;
			cellLeft.boundary.p1 = p1_left;
			cellRight.boundary.p0 = p0_right;
			cellRight.boundary.p1 = p1_right;
			cellLeft.edge = &edge;
			cellRight.edge = &edge;

			m_ECMCells.push_back(cellLeft);
			m_ECMCells.push_back(cellRight);

			// TODO: add cells to grid
		}
	}

	void ECMCellCollection::AddCellToGrid(ECMGraph& graph, const Point& gridBBOXMin, const ECMCell& cell)
	{
		// edge between bounding cells
		Point p0 = graph.GetVertex(cell.edge->half_edges[0].v_target_idx)->position;
		Point p1 = graph.GetVertex(cell.edge->half_edges[1].v_target_idx)->position;

		// calculate BBOX of cell and determine which grid cells overlap
		float minX = std::min({ p0.x, p1.x, cell.boundary.p0.x, cell.boundary.p1.x });
		float minY = std::min({ p0.y, p1.y, cell.boundary.p0.y, cell.boundary.p1.y });
		float maxX = std::max({ p0.x, p1.x, cell.boundary.p0.x, cell.boundary.p1.x });
		float maxY = std::max({ p0.y, p1.y, cell.boundary.p0.y, cell.boundary.p1.y });

		int minCol = (minX - gridBBOXMin.x) / m_GridCellSize;
		int maxCol = (maxX - gridBBOXMin.x) / m_GridCellSize;
		int minRow = (minY - gridBBOXMin.y) / m_GridCellSize;
		int maxRow = (maxY - gridBBOXMin.y) / m_GridCellSize;

		//Utility::MathUtility::
		// TODO: loop through the cells intersecting with BBOX. Per cell, test if it intersects (or lies in) the ECM cell.
	}



	void ECMCellCollection::Clear()
	{
		m_ECMCells.clear();
	}


	const ECMCell* ECMCellCollection::PointLocationQueryLinear(const ECMGraph& graph, const Point& location, int hintIdx) const
	{
		std::vector<Segment> polygon;
		polygon.resize(4);

		// first check if location lies in hintIdx cell
		if (hintIdx != -1)
		{
			const ECMCell* cell = graph.GetECMCell(hintIdx);

			const Segment& obstacle = cell->boundary;
			Point p1 = graph.GetVertex(cell->edge->half_edges[1].v_target_idx)->position;
			Point p2 = obstacle.p0;
			Point p3 = obstacle.p1;
			Point p4 = graph.GetVertex(cell->edge->half_edges[0].v_target_idx)->position;

			polygon[0].p0 = p1;
			polygon[0].p1 = p2;
			polygon[1].p0 = p2;
			polygon[1].p1 = p3;
			polygon[2].p0 = p3;
			polygon[2].p1 = p4;
			polygon[3].p0 = p4;
			polygon[3].p1 = p1;

			if (Utility::MathUtility::Contains(location, polygon))
			{
				return cell;
			}
		}

		for (const ECMCell& cell : m_ECMCells)
		{
			const Segment& obstacle = cell.boundary;
			Point p1 = graph.GetVertex(cell.edge->half_edges[1].v_target_idx)->position;
			Point p2 = obstacle.p0;
			Point p3 = obstacle.p1;
			Point p4 = graph.GetVertex(cell.edge->half_edges[0].v_target_idx)->position;

			polygon[0].p0 = p1;
			polygon[0].p1 = p2;
			polygon[1].p0 = p2;
			polygon[1].p1 = p3;
			polygon[2].p0 = p3;
			polygon[2].p1 = p4;
			polygon[3].p0 = p4;
			polygon[3].p1 = p1;
			
			if (Utility::MathUtility::Contains(location, polygon))
			{
				return &cell;
			}
		}

		printf("Couldn't locate point in the ECM graph!\n");
		return nullptr;
	}

}