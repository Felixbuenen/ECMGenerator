#include "ECMCellCollection.h"

#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"

namespace ECM {

	void ECMCellCollection::Construct(ECMGraph& graph)
	{
		for (const ECMEdge& edge : graph.GetEdges())
		{
			ECMCell cell;
			std::vector<Segment> cellBounds;

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

			cellBounds.push_back(Segment(p0, p0_left));
			cellBounds.push_back(Segment(p0_left, p1_left));
			cellBounds.push_back(Segment(p1_left, p1));
			cellBounds.push_back(Segment(p1, p1_right));
			cellBounds.push_back(Segment(p1_right, p0_right));
			cellBounds.push_back(Segment(p0_right, p0));

			cell.boundary = cellBounds;
			cell.ecmEdge = edge.idx;

			m_ECMCells.push_back(cell);
		}
	}


	// todo: make more clever (trapezoidal decomposition)
	ECMCell* ECMCellCollection::PointLocationQuery(const Point& location)
	{
		for (ECMCell& cell : m_ECMCells)
		{
			if (Utility::MathUtility::Contains(location, cell.boundary))
			{
				return &cell;
			}
		}

		printf("Couldn't locate point in the ECM graph!\n");
		return nullptr;
	}

}