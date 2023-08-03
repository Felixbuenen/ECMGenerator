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
struct geometry_concept<ECM::Point> { typedef point_concept type; };

template <>
struct point_traits<ECM::Point> {
	typedef int coordinate_type;

	static inline coordinate_type get(const ECM::Point& vertex, orientation_2d orient) {
		return (orient == HORIZONTAL) ? vertex.x : vertex.y;
	}
};

template <>
struct geometry_concept<ECM::Segment> { typedef segment_concept type; };

template <>
struct segment_traits<ECM::Segment> {
	typedef int coordinate_type;
	typedef ECM::Point point_type;

	static inline point_type get(const ECM::Segment& segment, direction_1d dir) {
		return dir.to_int() ? segment.p1 : segment.p0;
	}
};

namespace ECM {

	void ECMGenerator::ConstructECMGraph(ECM& ecm, const Environment& environment)
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

				float cost = Utility::MathUtility::Distance(x0, y0, x1, y1);
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

					Point closestP0 = Utility::MathUtility::GetClosestPointOnSegment(v0.Position(), source);
					Point closestP1 = Utility::MathUtility::GetClosestPointOnSegment(v1.Position(), source);

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

					Point closestP0 = Utility::MathUtility::GetClosestPointOnSegment(v0.Position(), source);
					Point closestP1 = Utility::MathUtility::GetClosestPointOnSegment(v1.Position(), source);

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

		// finally construct the cells
		ecmGraph.ConstructECMCells();
	}


	std::shared_ptr<ECM> ECMGenerator::GenerateECM(const Environment& environment)
	{
		std::shared_ptr<ECM> ecm = std::make_shared<ECM>();

		printf("Construct voronoi diagram (Boost)... ");
		boost::polygon::construct_voronoi(environment.GetEnvironmentObstacleUnion().begin(), environment.GetEnvironmentObstacleUnion().end(), &ecm->GetMedialAxis()->VD);

		printf("Done\n");

		printf("Construct ECM graph... ");
		// now we have the voronoi diagram en we can create the actual ecm graph
		ConstructECMGraph(*ecm, environment);
		printf("Done\n");

		return ecm;
	}

}