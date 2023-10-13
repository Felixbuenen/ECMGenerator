#include "ECMCellCollection.h"

#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "TrapezoidalDecomposition.h"

namespace ECM {

	void ECMCellCollection::Construct(ECMGraph& graph, ECMCellCollectionType type)
	{
		if (type == ECMCellCollectionType::TDC)
		{
			m_ECMCellDecomp = TrapezoidalDecomposition::Generate();
			return;
		}
		
		for (ECMEdge& edge : graph.GetEdges())
		{
			ECMCell cellLeft;
			ECMCell cellRight;

			const ECMHalfEdge& edge0 = edge.half_edges[0];
			const ECMHalfEdge& edge1 = edge.half_edges[1];

			ECMVertex* v0 = graph.GetVertex(edge.half_edges[1].v_target_idx);
			ECMVertex* v1 = graph.GetVertex(edge.half_edges[0].v_target_idx);

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
		}
	}

	ECMCell* ECMCellCollection::PointLocationQueryLinear(ECMGraph& graph, const Point& location)
	{
		//std::printf("%d\n", m_ECMCells.size());
		std::vector<Segment> polygon;
		polygon.resize(4);

		for (ECMCell& cell : m_ECMCells)
		{
			ECMVertex* v0 = graph.GetVertex(cell.edge->half_edges[0].v_target_idx);
			ECMVertex* v1 = graph.GetVertex(cell.edge->half_edges[1].v_target_idx);

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