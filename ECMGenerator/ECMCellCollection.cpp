#include "ECMCellCollection.h"

#include "UtilityFunctions.h"
#include "ECMDataTypes.h"
#include "ECM.h"

void ECMCellCollection::Construct(const ECMGraph& graph)
{
	for (const ECMEdge& edge : graph.GetEdges())
	{
		ECMCell cell;
		std::vector<Segment> cellBounds;

		Point v0 = graph.GetVertex(edge.V0()).Position();
		Point v1 = graph.GetVertex(edge.V1()).Position();

		cellBounds.push_back(Segment(v0, edge.NearestLeftV0()));
		cellBounds.push_back(Segment(edge.NearestLeftV0(), edge.NearestLeftV1()));
		cellBounds.push_back(Segment(edge.NearestLeftV1(), v1));
		cellBounds.push_back(Segment(v1, edge.NearestRightV1()));
		cellBounds.push_back(Segment(edge.NearestRightV1(), edge.NearestRightV0()));
		cellBounds.push_back(Segment(edge.NearestRightV0(), v0));

		cell.boundary = cellBounds;
		cell.ecmEdge = edge.Index();

		m_ECMCells.push_back(cell);
	}
}


// todo: make more clever (trapezoidal decomposition)
const ECMCell* ECMCellCollection::PointLocationQuery(const Point& location) const
{
	for (const ECMCell& cell : m_ECMCells)
	{
		if (MathUtility::Contains(location, cell.boundary))
		{
			return &cell;
		}
	}

	printf("Couldn't locate point in the ECM graph!\n");
	return nullptr;
}
