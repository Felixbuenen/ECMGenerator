#include "ECMGenerator.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "Environment.h"
#include "UtilityFunctions.h"

#include "boost/polygon/voronoi.hpp"

#include <cstddef>
#include <memory>
#include <map>


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
			// > use an acceleration structure
			// > maybe even implement this in the Boost library?
			ECMVertex vertex(vertLocation);
			std::vector<Point> closestPoints = environment.GetClosestObstaclePoints(vertLocation);
			for (const Point& p : closestPoints)
			{
				vertex.AddClosestPoint(p);
			}

			ecmGraph.AddVertex(vertex);
		}
	}

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

			if (v0_index == -1 || v1_index == -1) {
				continue;
			}

			// TODO: find v0 and v1 closest obstacle points using a KD-tree of the obstacles.

			float cost = MathUtility::Distance(x0, y0, x1, y1);
			
			int edgeIndex = ecmGraph.AddEdge(ECMEdge(v0_index, v1_index, cost));

			ecmGraph.AddAdjacency(v0_index, v1_index, edgeIndex);
		}
	}

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