#include "ECMGenerator.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "Environment.h"
#include "UtilityFunctions.h"

#include "boost/polygon/voronoi.hpp"

#include <cstddef>
#include <memory>
#include <map>

// TODO
// > Er zit een hoop boost code verwerkt in ECM code. Misschien een idee om de boost VD code + types te verbergen achter een ECMVoronoiDiagram class oid,
//   zodat de alleen de implementatie van de VD met Boost wordt gedaan. Dat is iets netter. 


// ---------- /this should all be refactored\ -------------
using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;
using boost::polygon::geometry_concept;
using boost::polygon::point_traits;
using boost::polygon::point_concept;
using boost::polygon::orientation_2d;
using boost::polygon::segment_concept;
using boost::polygon::direction_1d;
using boost::polygon::segment_traits;

namespace bp = boost::polygon;


template <>
struct geometry_concept<Point> { typedef point_concept type; };

template <>
struct point_traits<Point> {
	typedef int coordinate_type;

	static inline coordinate_type get(const Point& vertex, orientation_2d orient) {
		return (orient == HORIZONTAL) ? vertex.x : vertex.y;
	}
};

template <>
struct geometry_concept<Segment> { typedef segment_concept type; };

template <>
struct segment_traits<Segment> {
	typedef int coordinate_type;
	typedef Point point_type;

	static inline point_type get(const Segment& segment, direction_1d dir) {
		return dir.to_int() ? segment.p1 : segment.p0;
	}
};


// ---------- \this should all be refactored/ -------------

// ----------- \this is purely as an example/ -------------
// 
// Traversing Voronoi edges using edge iterator.
int iterate_primary_edges1(const voronoi_diagram<double>& vd) {
	int result = 0;
	for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
		it != vd.edges().end(); ++it) {
		if (it->is_primary())
			++result;
	}
	return result;
}

// Traversing Voronoi edges using cell iterator.
int iterate_primary_edges2(const voronoi_diagram<double>& vd) {
	int result = 0;
	for (voronoi_diagram<double>::const_cell_iterator it = vd.cells().begin();
		it != vd.cells().end(); ++it) {
		const voronoi_diagram<double>::cell_type& cell = *it;
		const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();
		// This is convenient way to iterate edges around Voronoi cell.
		do {
			if (edge->is_primary())
				++result;
			edge = edge->next();
		} while (edge != cell.incident_edge());
	}
	return result;
}

// Traversing Voronoi edges using vertex iterator.
// As opposite to the above two functions this one will not iterate through
// edges without finite endpoints and will iterate only once through edges
// with single finite endpoint.
int iterate_primary_edges3(const voronoi_diagram<double>& vd) {
	int result = 0;
	for (voronoi_diagram<double>::const_vertex_iterator it =
		vd.vertices().begin(); it != vd.vertices().end(); ++it) {
		const voronoi_diagram<double>::vertex_type& vertex = *it;
		const voronoi_diagram<double>::edge_type* edge = vertex.incident_edge();
		// This is convenient way to iterate edges around Voronoi vertex.
		do {
			if (edge->is_primary())
				++result;
			edge = edge->rot_next();
		} while (edge != vertex.incident_edge());
	}
	return result;
}
// ---------------------------------------------------------------------------------------------------------

void ECMGenerator::ConstructECMGraph(ECM& ecm, const Environment& environment) const
{
	const MedialAxis& ma = *ecm.GetMedialAxis();

	ECMGraph& ecmGraph = ecm.GetECMGraph();
	const boost::polygon::voronoi_diagram<double>& vd = ma.VD;
	
	// first create all ECM vertices
	for (voronoi_diagram<double>::const_vertex_iterator it = 
		vd.vertices().begin(); it != vd.vertices().end(); ++it) {
		const voronoi_diagram<double>::vertex_type& vertex = *it;

		Point vertLocation(vertex.x(), vertex.y());
		if (!environment.InsideObstacle(vertLocation))
		{
			// store the closest points in the vertex
			// TODO:
			// > In paper it rightfully states that this information could be added in constant time. After all, the VD is generated using 
			//   nearest obstacle information. We need to adjust the boost library for this.
			ECMVertex vertex(vertLocation);
			std::vector<Point> closestPoints = environment.GetClosestObstaclePoints(vertLocation);
			for (const Point& p : closestPoints)
			{
				vertex.AddClosestObstaclePoint(p);
			}

			ecmGraph.AddVertex(vertex);
		}
	}

	printf("Number of vertices: {%d}\n", ecmGraph.GetVertices().size());

	int counter = 0;
	// then create all ECM edges. Note that we do not have to check for edges inside obstacles. This is
	// automatically solved by not including vertices inside obstacles in the ecmGraph.
	for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
		it != vd.edges().end(); ++it) {
		
		if (it->is_primary() && it->is_finite())
		{
			float x0 = it->vertex0()->x();
			float x1 = it->vertex1()->x();
			float y0 = it->vertex0()->y();
			float y1 = it->vertex1()->y();

			int v0_index = ecmGraph.GetVertexIndex(x0, y0);
			int v1_index = ecmGraph.GetVertexIndex(x1, y1);

			// skip if a vertex could not be found
			if (v0_index == -1 || v1_index == -1) {
				continue;
			}

			// if there already exists an edge between v0 and v1, then we should not construct another edge (bi-directional).
			const std::vector<int>& edges = ecm.GetECMGraph().GetIncidentEdges(v1_index);
			bool edgeExists = false;
			for (int edge_index : edges)
			{
				if (ecm.GetECMGraph().GetEdge(edge_index).V1() == v0_index)
				{
					edgeExists = true;
					break;
				}
			}
			if (edgeExists) continue;

			// Given closest obstacle points per vertex, assign left/right v0 and v1
			// > For the segment, calculate the 'right' vector -> Vr.
			// > Using Vr, determine which closest obstacle points are left and right.
			// > If there is only 1 left/right closest point, assign this point.
			// > If there is more, then we want the obstacle point with the sharpest angle with v0-v1. This ensures
			//    that the segment is adjacent to the obstacle segment:
			// > for v0, take max(dot(obstaclePoint, segment)). 
			// > for v1, take min(dot(obstaclePoint, segment)). 
			//Vec2 right = MathUtility::Right()

			// TODO: it's important to determine how ECM will be traversed. If there is directionality (i.e. two edges for A->B and B->A), then
			// we can do the above. HOWEVER: the ECM is currently implemented bi-directional (i.e. 1 edge for A<->B). Now, there is no concept of left/right
			// and the above won't work. Then, you must determine 2 different sides of the line, and determine on runtime (during path planning) which one
			// is left and which one is right.

			
			float cost = MathUtility::Distance(x0, y0, x1, y1);
			ECMEdge edge(v0_index, v1_index, cost);
			
			// calculate nearest obstacle point information
			Vec2 edgeVec(x1 - x0, y1 - y0);
			float edgeVecT = MathUtility::SquaredLength(edgeVec);
			Vec2 rightVec = MathUtility::Right(edgeVec);
			float rightVecT = MathUtility::SquaredLength(rightVec);

			const std::vector<Point>& startClosestPoints = ecmGraph.GetVertex(v0_index).GetClosestPoints();
			const std::vector<Point>& endClosestPoints = ecmGraph.GetVertex(v1_index).GetClosestPoints();

			// 1. calculate closest points to start vertex
			//     Check for edge case when nearest obstacle positions equal the vertex position itself
			// TODO: make an 'estimate equal' function so we don't have to do this stuff everytime.
			bool startVertIntersectsObject = 
				x0 > startClosestPoints[0].x - ECM_EPSILON
				&& x0 < startClosestPoints[0].x + ECM_EPSILON
				&& y0 > startClosestPoints[0].y - ECM_EPSILON
				&& y0 < startClosestPoints[0].y + ECM_EPSILON;

			if (startVertIntersectsObject)
			{
				edge.SetNearestLeftV0(Point(x0, y0));
				edge.SetNearestRightV0(Point(x0, y0));
			}
			else
			{
				int idxLeft = -1, idxRight = -1;
				float largestDotLeft = -1000000.0f; // FIX THIS!!
				float largestDotRight = -1000000.0f; // FIX THIS!!

				int counter = 0;
				for (const Point& p : startClosestPoints)
				{
					Vec2 pVec(p.x - x0, p.y - y0);

					// because the length of the edge and the length to all closest points are constant, we only have to calculate the dot
					// product to determine which closest point makes the sharpest angle with the ecm edge.
					float dot = MathUtility::Dot(edgeVec, pVec);
					bool isLeft = MathUtility::Dot(rightVec, pVec) < 0;

					if (isLeft && dot > largestDotLeft)
					{
						// obstacle point on the left, and new sharpest angle wrt the ECM edge
						largestDotLeft = dot;
						idxLeft = counter;
						counter++;

						continue;
					}
					if (!isLeft && dot > largestDotRight)
					{
						// obstacle point on the right, and new largest angle wrt the ECM edge
						largestDotRight = dot;
						idxRight = counter;
						counter++;

						continue;
					}

					counter++;
				}

				if (idxLeft == -1 || idxRight == -1)
				{
					printf("ERROR: closest point could not be found!\n");
				}

				edge.SetNearestLeftV0(startClosestPoints[idxLeft]);
				edge.SetNearestRightV0(startClosestPoints[idxRight]);
			}

			bool endVertIntersectsObject =
				x1 > endClosestPoints[0].x - ECM_EPSILON
				&& x1 < endClosestPoints[0].x + ECM_EPSILON
				&& y1 > endClosestPoints[0].y - ECM_EPSILON
				&& y1 < endClosestPoints[0].y + ECM_EPSILON;

			if (endVertIntersectsObject)
			{
				edge.SetNearestLeftV1(Point(x1, y1));
				edge.SetNearestRightV1(Point(x1, y1));
			}
			else
			{
				int idxLeft = -1, idxRight = -1;
				float smallestDotLeft = 10000000.0f; // FIX THIS!!
				float smallestDotRight = 10000000.0f; // FIX THIS!!
				
				int counter = 0;
				for (const Point& p : endClosestPoints)
				{
					Vec2 pVec(p.x - x1, p.y - y1);
				
					// because the length of the edge and the length to all closest points are constant, we only have to calculate the dot
					// product to determine which closest point makes the sharpest angle with the ecm edge.
					float dot = MathUtility::Dot(edgeVec, pVec);
					bool isLeft = MathUtility::Dot(rightVec, pVec) < 0;
				
					if (isLeft && dot < smallestDotLeft)
					{
						smallestDotLeft = dot;
						idxLeft = counter;
						counter++;
				
						continue;
					}
					if (!isLeft && dot < smallestDotRight)
					{
						smallestDotRight = dot;
						idxRight = counter;
						counter++;
				
						continue;
					}
				
					counter++;
				}
				
				if (idxLeft == -1 || idxRight == -1)
				{
					printf("ERROR: closest point could not be found!\n");
				}

				edge.SetNearestLeftV1(endClosestPoints[idxLeft]);
				edge.SetNearestRightV1(endClosestPoints[idxRight]);
			}

			int edgeIndex = ecmGraph.AddEdge(edge);
			ecmGraph.AddAdjacency(v0_index, v1_index, edgeIndex);

			counter++;
		}
	}

	printf("number of edges: %d\n", counter);

	// finally construct the cells
	ecmGraph.ConstructECMCells();
}


std::shared_ptr<ECM> ECMGenerator::GenerateECM(const Environment& environment) const
{
	std::shared_ptr<ECM> ecm = std::make_shared<ECM>();

	// first make union of walkable area and obstacles
	std::vector<Segment> envUnion;
	for (const Segment& seg : environment.GetWalkableArea())
	{
		envUnion.push_back(seg);
	}
	for (auto obstacle : environment.GetObstacles())
	{
		for (const Segment& seg : obstacle)
		{
			envUnion.push_back(seg);
		}
	}

	printf("Construct voronoi diagram (Boost)... ");
	boost::polygon::construct_voronoi(envUnion.begin(), envUnion.end(), &ecm->GetMedialAxis()->VD);
	printf("Done\n");

	printf("Construct ECM graph... ");
	// now we have the voronoi diagram en we can create the actual ecm graph
	ConstructECMGraph(*ecm, environment);
	printf("Done\n");

	ECMGraph& graph = ecm->GetECMGraph();
	

	// Traversing Voronoi Graph.

	//printf("Traversing Voronoi graph.\n");
	//printf("Number of visited primary edges using edge iterator: %d\n",
	//	iterate_primary_edges1(ecm->GetMedialAxis()->VD));
	//printf("Number of visited primary edges using cell iterator: %d\n",
	//	iterate_primary_edges2(ecm->GetMedialAxis()->VD));
	//printf("Number of visited primary edges using vertex iterator: %d\n",
	//	iterate_primary_edges3(ecm->GetMedialAxis()->VD));
	//printf("\n");

	

	//for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
	//	it != vd.edges().end(); ++it) {
	//	if (it->is_primary() && it->is_linear() && it->is_finite())
	//	{
	//		++
	//	}
	//}

	// Traverse VD segments
	//int counter = 0;
	//for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
	//	it != vd.edges().end(); ++it) {
	//	if (it->is_primary() && it->is_finite())
	//	{
	//		++counter;
	//		const double x1 = it->vertex0()->x();
	//		const double y1 = it->vertex0()->y();
	//		const double x2 = it->vertex1()->x();
	//		const double y2 = it->vertex1()->y();
	//		printf("Segment %d: (%f, %f) -> (%f, %f)\n", counter, x1, y1, x2, y2);
	//	}
	//
	//	if (it->is_primary() && it->is_infinite())
	//	{
	//		// p1 or/and(?) p2 is nullptr, so we cant access that.
	//
	//		++counter;
	//		//const double x1 = it->vertex0()->x();
	//		//const double y1 = it->vertex0()->y();
	//		//const double x2 = it->vertex1()->x();
	//		//const double y2 = it->vertex1()->y();
	//		//printf("Segment %d: (%f, %f) -> (%f, %f)\n", counter, x1, y1, x2, y2);
	//	}
	//}



	return ecm;
}