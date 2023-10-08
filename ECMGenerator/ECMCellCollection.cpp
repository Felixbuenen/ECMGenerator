#include "ECMCellCollection.h"

#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"

namespace ECM {

	void ECMCellCollection::Construct(ECMGraph& graph)
	{
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


	// todo: make more clever (trapezoidal decomposition)
	ECMCell* ECMCellCollection::PointLocationQuery(ECMGraph& graph, const Point& location)
	{
		for (ECMCell& cell : m_ECMCells)
		{
			ECMVertex* v0 = graph.GetVertex(cell.edge->half_edges[0].v_target_idx);
			ECMVertex* v1 = graph.GetVertex(cell.edge->half_edges[1].v_target_idx);

			const Segment& obstacle = cell.boundary;
			Point p1 = graph.GetVertex(cell.edge->half_edges[1].v_target_idx)->position;
			Point p2 = obstacle.p0;
			Point p3 = obstacle.p1;
			Point p4 = graph.GetVertex(cell.edge->half_edges[0].v_target_idx)->position;

			std::vector<Segment> polygon;
			polygon.push_back(Segment(p1, p2));
			polygon.push_back(Segment(p2, p3));
			polygon.push_back(Segment(p3, p4));
			polygon.push_back(Segment(p4, p1));
			
			if (Utility::MathUtility::Contains(location, polygon))
			{
				return &cell;
			}
		}

		printf("Couldn't locate point in the ECM graph!\n");
		return nullptr;
	}

}