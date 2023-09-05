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

		// TODO: 
		// 1. Loop through vertices. Add all vertices that are not inside an obstacle.
		// 2. Loop through edges. Add edges and respective half edges. Store an array of indices of size boostEdges.
		//    In this array A, store the index of our ECM half edge, at the index of the boost edge. I.e. A[boost_edge_id] = ECMedgeID.
		// 3. Loop through vertices again. This time, add next_incident_edge information. Use A to set the correct ecm edge ID
		// 
		// first create all ECM vertices

		std::printf("Adding vertices... ");
		std::vector<int> mapECMVertexIndices;
		mapECMVertexIndices.resize(vd.num_vertices());
		int boostVertIndex = -1;
		int ecmVertIndex = -1;
		for (voronoi_diagram<double>::const_vertex_iterator it =
			vd.vertices().begin(); it != vd.vertices().end(); ++it) {

			boostVertIndex++;
			
			Point vertLocation(it->x(), it->y());

			// don't add vertex if it lies inside obstacle
			if (environment.InsideObstacle(vertLocation)) continue;
			
			// don't add vertex if all its incident edges lie inside obstacle
			const voronoi_diagram<double>::edge_type* edge = it->incident_edge();
			bool shouldAdd = false;
			do {
				if (edge->is_primary())
				{
					Point loc;
					loc.x = edge->vertex1()->x();
					loc.y = edge->vertex1()->y();
					
					if (!environment.InsideObstacle(loc))
					{
						shouldAdd = true;
						break;
					}
				}
				edge = edge->rot_next();
			} while (edge != it->incident_edge());

			if (!shouldAdd) continue;
			
			ecmVertIndex++;
			ecmGraph.AddVertex(vertLocation);
			mapECMVertexIndices[ecmVertIndex] = boostVertIndex;
		}
		std::printf("Done\n");


		std::printf("Adding edges... ");

		std::vector<int> mapECMEdgeIndices;
		mapECMEdgeIndices.resize(vd.num_edges(), -1);
		int boostEdgeIndex = -1;
		int ecmEdgeIndex = -1;
		std::vector<bool> handled; 
		handled.resize(vd.num_edges());
		for (voronoi_diagram<double>::const_edge_iterator it = vd.edges().begin();
			it != vd.edges().end(); ++it) {
			boostEdgeIndex++;

			if (handled[boostEdgeIndex]) continue;

			if (it->is_primary() && it->is_finite())
			{
				Point p1 = Point(it->vertex0()->x(), it->vertex0()->y());
				Point p2 = Point(it->vertex1()->x(), it->vertex1()->y());

				int v0_index = ecmGraph.FindVertex(p1.x, p1.y);
				int v1_index = ecmGraph.FindVertex(p2.x, p2.y);

				// skip if a vertex could not be found
				if (v0_index == -1 || v1_index == -1) {
					continue;
				}

				if (v0_index == 4 && v1_index == 2)
				{
					bool stop = true;
				}

				ECMEdge* edge = ecmGraph.AddEdge();

				// check closest points to left
				Point closestLeft1, closestLeft2;
				int srcIdx = it->cell()->source_index();
				bool isPoint = it->cell()->contains_point();
				SourceCategory cat1 = it->cell()->source_category();
				bool isStartPoint = it->cell()->source_category() == SourceCategory::SOURCE_CATEGORY_SEGMENT_START_POINT;
				GetClosestPointsToSource(environment, srcIdx, p1, p2, isPoint, isStartPoint, closestLeft1, closestLeft2);

				// check closest points to right
				Point closestRight1, closestRight2;
				srcIdx = it->twin()->cell()->source_index();
				isPoint = it->twin()->cell()->contains_point();
				SourceCategory cat2 = it->cell()->source_category();
				isStartPoint = it->twin()->cell()->source_category() == SourceCategory::SOURCE_CATEGORY_SEGMENT_START_POINT;
				GetClosestPointsToSource(environment, srcIdx, p1, p2, isPoint, isStartPoint, closestRight1, closestRight2);

				// construct half-edges
				ecmGraph.AddHalfEdge(edge->idx, v1_index, closestLeft1, closestRight1, 0);
				ecmGraph.AddHalfEdge(edge->idx, v0_index, closestRight2, closestLeft2, 1);

				// bookkeeping to reference boost edges later
				ecmEdgeIndex++;
				mapECMEdgeIndices[boostEdgeIndex] = ecmEdgeIndex;

				// remove twin of this half-edge -> we already added it
				int twinIdx = it->twin() - &vd.edges()[0];
				handled[twinIdx] = true;
				mapECMEdgeIndices[twinIdx] = ecmEdgeIndex;
			}
		}
		std::printf("Done\n");



		std::printf("add incident edge info...");

		// add incident edge info
		const auto& boostVerts = vd.vertices();
		for (int i = 0; i < ecmGraph.GetVertices().size(); i++) {
			int boostVertIdx = mapECMVertexIndices[i];
			auto incEdge = boostVerts[boostVertIdx].incident_edge();
			int boostEdgeIdx = incEdge - &vd.edges()[0];
			int ecmEdgeIdx = mapECMEdgeIndices[boostEdgeIdx];
			while (ecmEdgeIdx == -1)
			{
				incEdge = boostVerts[boostVertIdx].incident_edge()->rot_next();
				boostEdgeIdx = incEdge - &vd.edges()[0];
				ecmEdgeIdx = mapECMEdgeIndices[boostEdgeIdx];
			}

			ECMEdge* edge = ecmGraph.GetEdge(ecmEdgeIdx);
			int halfEdgeOffset = edge->half_edges[0].v_target_idx == i ? 1 : 0;
			ECMHalfEdge* halfEdge = &edge->half_edges[halfEdgeOffset];

			ECMVertex* vert = ecmGraph.GetVertex(i);
			vert->half_edge_idx = ecmEdgeIdx * 2 + halfEdgeOffset; // refactor?

			int startHalfEdgeIdx = vert->half_edge_idx;
			int nextHalfEdgeIndex = startHalfEdgeIdx;

			auto* e = &vd.edges()[boostEdgeIdx];
			auto* eStart = e;
			do {
				e = e->rot_next();
				int nextBoostEdgeIdx = e - &vd.edges()[0];
				int nextEcmEdgeIdx = mapECMEdgeIndices[nextBoostEdgeIdx];
				if (nextEcmEdgeIdx == -1)
				{
					continue;
				}

				ECMEdge* nextEdge = ecmGraph.GetEdge(nextEcmEdgeIdx);
				int nextHalfEdgeOffset = nextEdge->half_edges[0].v_target_idx == i ? 1 : 0;
				nextHalfEdgeIndex = nextEcmEdgeIdx * 2 + nextHalfEdgeOffset;

				halfEdge->next_idx = nextHalfEdgeIndex;
				halfEdge = &nextEdge->half_edges[nextHalfEdgeOffset];
			} while (e != eStart);
		
		}
		std::printf("Done\n");

		std::printf("construct ecm cells...");

		// finally construct the cells
		ecmGraph.ConstructECMCells();

		std::printf("Done\n");

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

	void ECMGenerator::GetClosestPointsToSource(const Environment& environment, int sourceIdx, const Point& p1, const Point& p2, bool isPoint, bool isStartPoint, Point& outClosestP1, Point& outClosestP2)
	{
		Segment source = environment.GetEnvironmentObstacleUnion()[sourceIdx];

		if (isPoint)
		{
			Point p = isStartPoint ? source.p0 : source.p1;

			outClosestP1 = p;
			outClosestP2 = p;
		}
		else
		{
			Point closestP1 = Utility::MathUtility::GetClosestPointOnSegment(p1, source);
			Point closestP2 = Utility::MathUtility::GetClosestPointOnSegment(p2, source);
			
			outClosestP1 = closestP1;
			outClosestP2 = closestP2;
		}
	}
}