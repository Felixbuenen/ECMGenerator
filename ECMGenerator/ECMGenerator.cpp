#include "ECMGenerator.h"
#include "ECMDataTypes.h"
#include "ECM.h"
#include "Environment.h"
#include "UtilityFunctions.h"

#include "boost/polygon/voronoi.hpp"
#include "BoostVisualizeUtils.h"
#include <boost/polygon/polygon.hpp>

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
using boost::polygon::SourceCategory;


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

		Point vertLocation(it->x(), it->y());
		if (environment.InsideObstacle(vertLocation)) continue;

		ECMVertex vertex(vertLocation);
		ecmGraph.AddVertex(vertex);
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

			ECMVertex v0 = ecmGraph.GetVertex(v0_index);
			ECMVertex v1 = ecmGraph.GetVertex(v1_index);

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
			
			float cost = MathUtility::Distance(x0, y0, x1, y1);
			ECMEdge edge(v0_index, v1_index, cost);

			// set whether or not the edge is a parabolic arc
			edge.SetIsArc(it->is_curved());

			// add closest obstacle information
			// the cell incident (left) to the half-edge of the current iterator contains the closest obstacle information
			// assume that we only have segments as input (so point must be from input segment)

			// TODO: refactor (lots of code duplication)
			// left side contains point
			if (it->cell()->contains_point())
			{
				int sourceIdx = it->cell()->source_index();
				Segment source = environment.GetEnvironmentObstacleUnion()[sourceIdx];
				Point p = it->cell()->source_category() == SourceCategory::SOURCE_CATEGORY_SEGMENT_START_POINT ? source.p0 : source.p1;
				
				v0.AddClosestObstaclePoint(p);
				v1.AddClosestObstaclePoint(p);

				edge.SetNearestLeftV0(p);
				edge.SetNearestLeftV1(p);
			}
			// left side contains segment
			else
			{
				int sourceIdx = it->cell()->source_index();
				Segment source = environment.GetEnvironmentObstacleUnion()[sourceIdx];
				
				Point closestP0 = MathUtility::GetClosestPointOnSegment(v0.Position(), source);
				Point closestP1 = MathUtility::GetClosestPointOnSegment(v1.Position(), source);

				v0.AddClosestObstaclePoint(closestP0);
				v1.AddClosestObstaclePoint(closestP1);

				edge.SetNearestLeftV0(closestP0);
				edge.SetNearestLeftV1(closestP1);
			}
			// right side contains point
			if (it->twin()->cell()->contains_point())
			{
				int sourceIdx = it->twin()->cell()->source_index();
				Segment source = environment.GetEnvironmentObstacleUnion()[sourceIdx];
				Point p = it->twin()->cell()->source_category() == SourceCategory::SOURCE_CATEGORY_SEGMENT_START_POINT ? source.p0 : source.p1;

				v0.AddClosestObstaclePoint(p);
				v1.AddClosestObstaclePoint(p);

				edge.SetNearestRightV0(p);
				edge.SetNearestRightV1(p);
			}
			// right side contains segment
			else
			{
				int sourceIdx = it->twin()->cell()->source_index();
				Segment source = environment.GetEnvironmentObstacleUnion()[sourceIdx];

				Point closestP0 = MathUtility::GetClosestPointOnSegment(v0.Position(), source);
				Point closestP1 = MathUtility::GetClosestPointOnSegment(v1.Position(), source);

				v0.AddClosestObstaclePoint(closestP0);
				v1.AddClosestObstaclePoint(closestP1);

				edge.SetNearestRightV0(closestP0);
				edge.SetNearestRightV1(closestP1);
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

	printf("Construct voronoi diagram (Boost)... ");
	boost::polygon::construct_voronoi(environment.GetEnvironmentObstacleUnion().begin(), environment.GetEnvironmentObstacleUnion().end(), &ecm->GetMedialAxis()->VD);
	
	printf("Done\n");

	printf("Construct ECM graph... ");
	// now we have the voronoi diagram en we can create the actual ecm graph
	ConstructECMGraph(*ecm, environment);
	printf("Done\n");

	ECMGraph& graph = ecm->GetECMGraph();
	/*
	using namespace boost::polygon;
	typedef double coordinate_type;
	typedef boost::polygon::point_data<coordinate_type> point_type;
	typedef boost::polygon::segment_data<coordinate_type> segment_type;

	for (const ECMEdge& e : graph.GetEdges())
	{
		if (e.IsArc())
		{
			if (MathUtility::SquareDistance(e.NearestLeftV0(), e.NearestLeftV1()) < ECM_EPSILON)
			{
				point_type p = e.NearestLeftV0();
				segment_type seg(e.NearestRightV0(), e.NearestRightV1());
				const double maxDist = 0.1;

				std::vector<point_type> discr;
				discr.push_back(graph.GetVertex(e.V0()).Position());
				discr.push_back(graph.GetVertex(e.V1()).Position());

				//auto c = boost::polygon::voronoi_visual_utils<double>::discretize(p, s, maxDist, &discr);
				boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(p, seg, maxDist, &discr);
			}
			else
			{
				point_type p = e.NearestRightV0();
				segment_type seg(e.NearestLeftV0(), e.NearestLeftV1());
				const double maxDist = 0.1;

				std::vector<point_type> discr;
				discr.push_back(graph.GetVertex(e.V0()).Position());
				discr.push_back(graph.GetVertex(e.V1()).Position());

				//auto c = boost::polygon::voronoi_visual_utils<double>::discretize(p, s, maxDist, &discr);
				boost::polygon::voronoi_visual_utils<coordinate_type>::discretize(p, seg, maxDist, &discr);
			}

			break;
		}
	}
	*/

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